#!/usr/bin/env python3
"""
Verify Armstrong project setup is correct
"""

import os
import sys
from pathlib import Path

def check_file_exists(path, description):
    """Check if a file exists and print status"""
    if Path(path).exists():
        print(f"✅ {description}: {path}")
        return True
    else:
        print(f"❌ {description}: {path} (MISSING)")
        return False

def check_directory_structure():
    """Check that all required directories exist"""
    print("🔍 Checking directory structure...")
    
    required_dirs = [
        "workspace/src/ur5_gazebo",
        "workspace/src/ur5_moveit_config", 
        "workspace/src/bc_training",
        "data/demonstrations",
        "data/models",
        "data/results",
        "gazebo_models"
    ]
    
    all_good = True
    for dir_path in required_dirs:
        if Path(dir_path).is_dir():
            print(f"✅ Directory: {dir_path}")
        else:
            print(f"❌ Directory: {dir_path} (MISSING)")
            all_good = False
    
    return all_good

def check_package_files():
    """Check that package files are properly configured"""
    print("\n🔍 Checking package configuration files...")
    
    files_to_check = [
        ("workspace/src/ur5_gazebo/package.xml", "UR5 Gazebo package.xml"),
        ("workspace/src/ur5_gazebo/setup.py", "UR5 Gazebo setup.py"),
        ("workspace/src/ur5_gazebo/CMakeLists.txt", "UR5 Gazebo CMakeLists.txt"),
        ("workspace/src/ur5_moveit_config/package.xml", "MoveIt package.xml"),
        ("workspace/src/ur5_moveit_config/CMakeLists.txt", "MoveIt CMakeLists.txt"),
        ("workspace/src/bc_training/package.xml", "BC Training package.xml"),
        ("workspace/src/bc_training/setup.py", "BC Training setup.py"),
    ]
    
    all_good = True
    for file_path, description in files_to_check:
        if not check_file_exists(file_path, description):
            all_good = False
    
    return all_good

def check_launch_files():
    """Check launch files exist"""
    print("\n🔍 Checking launch files...")
    
    launch_files = [
        ("workspace/src/ur5_gazebo/launch/ur5.launch.py", "UR5 Gazebo launch file"),
        ("workspace/src/ur5_moveit_config/launch/moveit.launch.py", "MoveIt launch file"),
        ("workspace/src/ur5_moveit_config/launch/rviz.launch.py", "RViz launch file"),
    ]
    
    all_good = True
    for file_path, description in launch_files:
        if not check_file_exists(file_path, description):
            all_good = False
    
    return all_good

def check_config_files():
    """Check configuration files"""
    print("\n🔍 Checking configuration files...")
    
    config_files = [
        ("workspace/src/ur5_moveit_config/config/ur5.srdf", "SRDF file"),
        ("workspace/src/ur5_moveit_config/config/kinematics.yaml", "Kinematics config"),
        ("workspace/src/ur5_moveit_config/config/joint_limits.yaml", "Joint limits config"),
        ("workspace/src/ur5_moveit_config/config/ompl_planning.yaml", "OMPL planning config"),
        ("workspace/src/ur5_moveit_config/config/moveit.rviz", "RViz config"),
    ]
    
    all_good = True
    for file_path, description in config_files:
        if not check_file_exists(file_path, description):
            all_good = False
    
    return all_good

def check_scripts():
    """Check Python scripts"""
    print("\n🔍 Checking Python scripts...")
    
    scripts = [
        ("workspace/src/ur5_gazebo/scripts/camera_detector.py", "Camera detector script"),
        ("workspace/src/ur5_gazebo/scripts/pick_place_controller.py", "Pick place controller"),
        ("workspace/src/ur5_gazebo/scripts/demo_recorder.py", "Demo recorder"),
        ("workspace/src/ur5_gazebo/scripts/bc_policy_controller.py", "BC policy controller"),
        ("workspace/src/ur5_gazebo/scripts/comparison_evaluator.py", "Comparison evaluator"),
        ("workspace/src/bc_training/train_policy.py", "BC training script"),
    ]
    
    all_good = True
    for file_path, description in scripts:
        if not check_file_exists(file_path, description):
            all_good = False
    
    return all_good

def check_resource_files():
    """Check package resource files"""
    print("\n🔍 Checking package resource files...")
    
    resource_files = [
        ("workspace/src/ur5_gazebo/resource/ur5_gazebo", "UR5 Gazebo resource file"),
        ("workspace/src/ur5_gazebo/ur5_gazebo/__init__.py", "UR5 Gazebo __init__.py"),
        ("workspace/src/bc_training/resource/bc_training", "BC Training resource file"),
        ("workspace/src/bc_training/bc_training/__init__.py", "BC Training __init__.py"),
    ]
    
    all_good = True
    for file_path, description in resource_files:
        if not check_file_exists(file_path, description):
            all_good = False
    
    return all_good

def check_urdf_world_files():
    """Check URDF and world files"""
    print("\n🔍 Checking URDF and world files...")
    
    files = [
        ("workspace/src/ur5_gazebo/urdf/ur5.urdf.xacro", "UR5 URDF file"),
        ("workspace/src/ur5_gazebo/worlds/pick_place_world.world", "Pick place world file"),
    ]
    
    all_good = True
    for file_path, description in files:
        if not check_file_exists(file_path, description):
            all_good = False
    
    return all_good

def check_docker_files():
    """Check Docker configuration"""
    print("\n🔍 Checking Docker configuration...")
    
    files = [
        ("docker-compose.yml", "Docker Compose file"),
        ("run_experiment.sh", "Experiment runner script"),
        ("workspace/requirements.txt", "Python requirements"),
    ]
    
    all_good = True
    for file_path, description in files:
        if not check_file_exists(file_path, description):
            all_good = False
    
    # Check if experiment script is executable
    if Path("run_experiment.sh").exists():
        if os.access("run_experiment.sh", os.X_OK):
            print("✅ run_experiment.sh is executable")
        else:
            print("❌ run_experiment.sh is not executable")
            all_good = False
    
    return all_good

def main():
    """Main verification function"""
    print("🚀 Armstrong Project Setup Verification")
    print("=" * 50)
    
    # Change to project root
    os.chdir(Path(__file__).parent)
    
    checks = [
        check_directory_structure,
        check_package_files,
        check_launch_files,
        check_config_files,
        check_scripts,
        check_resource_files,
        check_urdf_world_files,
        check_docker_files,
    ]
    
    all_passed = True
    for check in checks:
        if not check():
            all_passed = False
    
    print("\n" + "=" * 50)
    if all_passed:
        print("🎉 All checks passed! Setup looks good.")
        print("\nNext steps:")
        print("1. Run: docker-compose up -d")
        print("2. Run: ./run_experiment.sh")
    else:
        print("❌ Some checks failed. Please fix the issues above.")
        sys.exit(1)

if __name__ == "__main__":
    main()