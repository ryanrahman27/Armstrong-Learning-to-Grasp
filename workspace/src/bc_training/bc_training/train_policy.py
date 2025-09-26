#!/usr/bin/env python3

import os
import numpy as np
import tensorflow as tf
from tensorflow import keras
from tensorflow.keras import layers
import rosbag2_py
from rclpy.serialization import deserialize_message
from builtin_interfaces.msg import Time
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
import pickle
import argparse


class BehavioralCloningTrainer:
    def __init__(self, bag_path, model_save_path='bc_model.h5'):
        self.bag_path = bag_path
        self.model_save_path = model_save_path
        self.demonstrations = []
        
    def load_demonstrations(self):
        """Load demonstrations from ROS bag"""
        print(f"Loading demonstrations from {self.bag_path}")
        
        # Create reader
        reader = rosbag2_py.SequentialReader()
        storage_options = rosbag2_py.StorageOptions(uri=self.bag_path, storage_id='sqlite3')
        converter_options = rosbag2_py.ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr'
        )
        reader.open(storage_options, converter_options)
        
        # Get topic metadata
        topic_types = reader.get_all_topics_and_types()
        type_map = {topic.name: topic.type for topic in topic_types}
        
        # Process messages
        joint_states = []
        block_poses = []
        
        while reader.has_next():
            (topic, data, timestamp) = reader.read_next()
            
            if topic == '/joint_states':
                msg = deserialize_message(data, JointState)
                joint_states.append({
                    'timestamp': timestamp,
                    'positions': msg.position,
                    'velocities': msg.velocity
                })
            elif topic == '/block_pose':
                msg = deserialize_message(data, PoseStamped)
                block_poses.append({
                    'timestamp': timestamp,
                    'position': [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z],
                    'orientation': [msg.pose.orientation.x, msg.pose.orientation.y, 
                                  msg.pose.orientation.z, msg.pose.orientation.w]
                })
        
        reader.close()
        
        # Synchronize and create training pairs
        self.create_training_pairs(joint_states, block_poses)
        print(f"Loaded {len(self.demonstrations)} demonstration pairs")
    
    def create_training_pairs(self, joint_states, block_poses):
        """Create synchronized input-output pairs for training"""
        for i, js in enumerate(joint_states[:-1]):  # Exclude last state
            # Find closest block pose
            closest_block = None
            min_time_diff = float('inf')
            
            for bp in block_poses:
                time_diff = abs(bp['timestamp'] - js['timestamp'])
                if time_diff < min_time_diff:
                    min_time_diff = time_diff
                    closest_block = bp
            
            if closest_block and min_time_diff < 1e8:  # 100ms threshold
                # Input: current joint state + block pose
                input_state = list(js['positions']) + closest_block['position'] + closest_block['orientation']
                
                # Output: next joint state (action)
                next_js = joint_states[i + 1]
                output_action = list(next_js['positions'])
                
                self.demonstrations.append({
                    'input': input_state,
                    'output': output_action
                })
    
    def prepare_data(self):
        """Prepare training data"""
        if not self.demonstrations:
            raise ValueError("No demonstrations loaded")
        
        # Convert to numpy arrays
        X = np.array([demo['input'] for demo in self.demonstrations])
        y = np.array([demo['output'] for demo in self.demonstrations])
        
        # Normalize data
        self.input_mean = np.mean(X, axis=0)
        self.input_std = np.std(X, axis=0) + 1e-8
        self.output_mean = np.mean(y, axis=0)
        self.output_std = np.std(y, axis=0) + 1e-8
        
        X_norm = (X - self.input_mean) / self.input_std
        y_norm = (y - self.output_mean) / self.output_std
        
        # Train/validation split
        split_idx = int(0.8 * len(X_norm))
        
        self.X_train = X_norm[:split_idx]
        self.y_train = y_norm[:split_idx]
        self.X_val = X_norm[split_idx:]
        self.y_val = y_norm[split_idx:]
        
        print(f"Training data shape: {self.X_train.shape}")
        print(f"Training labels shape: {self.y_train.shape}")
        
        return self.X_train, self.y_train, self.X_val, self.y_val
    
    def build_model(self, input_dim, output_dim):
        """Build neural network model"""
        model = keras.Sequential([
            layers.Dense(256, activation='relu', input_shape=(input_dim,)),
            layers.Dropout(0.2),
            layers.Dense(256, activation='relu'),
            layers.Dropout(0.2),
            layers.Dense(128, activation='relu'),
            layers.Dropout(0.2),
            layers.Dense(output_dim, activation='linear')
        ])
        
        model.compile(
            optimizer='adam',
            loss='mse',
            metrics=['mae']
        )
        
        return model
    
    def train(self, epochs=100, batch_size=32):
        """Train the behavioral cloning model"""
        # Prepare data
        X_train, y_train, X_val, y_val = self.prepare_data()
        
        # Build model
        input_dim = X_train.shape[1]
        output_dim = y_train.shape[1]
        model = self.build_model(input_dim, output_dim)
        
        print("Model architecture:")
        model.summary()
        
        # Callbacks
        callbacks = [
            keras.callbacks.EarlyStopping(
                monitor='val_loss',
                patience=10,
                restore_best_weights=True
            ),
            keras.callbacks.ReduceLROnPlateau(
                monitor='val_loss',
                factor=0.5,
                patience=5
            )
        ]
        
        # Train model
        history = model.fit(
            X_train, y_train,
            validation_data=(X_val, y_val),
            epochs=epochs,
            batch_size=batch_size,
            callbacks=callbacks,
            verbose=1
        )
        
        # Save model and normalization parameters
        model.save(self.model_save_path)
        
        norm_params = {
            'input_mean': self.input_mean,
            'input_std': self.input_std,
            'output_mean': self.output_mean,
            'output_std': self.output_std
        }
        
        with open(self.model_save_path.replace('.h5', '_norm.pkl'), 'wb') as f:
            pickle.dump(norm_params, f)
        
        print(f"Model saved to {self.model_save_path}")
        
        # Final evaluation
        test_loss, test_mae = model.evaluate(X_val, y_val, verbose=0)
        print(f"Final test loss: {test_loss:.6f}")
        print(f"Final test MAE: {test_mae:.6f}")
        
        return model, history
    
    def evaluate_model(self, model_path=None):
        """Evaluate trained model"""
        if model_path is None:
            model_path = self.model_save_path
        
        # Load model and normalization
        model = keras.models.load_model(model_path)
        
        with open(model_path.replace('.h5', '_norm.pkl'), 'rb') as f:
            norm_params = pickle.load(f)
        
        # Prepare test data
        self.input_mean = norm_params['input_mean']
        self.input_std = norm_params['input_std']
        self.output_mean = norm_params['output_mean']
        self.output_std = norm_params['output_std']
        
        X_train, y_train, X_val, y_val = self.prepare_data()
        
        # Predictions
        y_pred = model.predict(X_val)
        
        # Denormalize
        y_val_denorm = y_val * self.output_std + self.output_mean
        y_pred_denorm = y_pred * self.output_std + self.output_mean
        
        # Calculate metrics
        mse = np.mean((y_val_denorm - y_pred_denorm) ** 2)
        mae = np.mean(np.abs(y_val_denorm - y_pred_denorm))
        
        print(f"Denormalized MSE: {mse:.6f}")
        print(f"Denormalized MAE: {mae:.6f}")
        
        return mse, mae


def main():
    parser = argparse.ArgumentParser(description='Train behavioral cloning policy')
    parser.add_argument('--bag_path', type=str, required=True,
                       help='Path to ROS bag with demonstrations')
    parser.add_argument('--model_path', type=str, default='bc_model.h5',
                       help='Path to save trained model')
    parser.add_argument('--epochs', type=int, default=100,
                       help='Number of training epochs')
    parser.add_argument('--batch_size', type=int, default=32,
                       help='Training batch size')
    parser.add_argument('--evaluate', action='store_true',
                       help='Evaluate existing model instead of training')
    
    args = parser.parse_args()
    
    trainer = BehavioralCloningTrainer(args.bag_path, args.model_path)
    
    if args.evaluate:
        trainer.load_demonstrations()
        trainer.evaluate_model(args.model_path)
    else:
        trainer.load_demonstrations()
        model, history = trainer.train(args.epochs, args.batch_size)


if __name__ == '__main__':
    main()