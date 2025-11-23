#!/usr/bin/env python3
"""
IMX219 Camera Initialization Script for XleRobot
"""
import os
import sys
import time
import subprocess

def load_kernel_modules():
    """Load required kernel modules"""
    modules = [
        "imx219",
        "hobot_mipicsi",
        "hobot_vin_vnode",
        "hobot_vin_vcon",
        "hobot_camsys",
        "hobot_sensor"
    ]

    for module in modules:
        try:
            subprocess.run(["sudo", "modprobe", module], check=True, capture_output=True)
            print(f"✓ Module {module} loaded successfully")
        except subprocess.CalledProcessError as e:
            print(f"✗ Module {module} failed to load: {e}")

def reset_camera_gpio():
    """Reset camera using GPIO"""
    try:
        # Run our fixed camera reset script
        result = subprocess.run([
            "sudo", "/usr/bin/python3",
            "/home/sunrise/xlerobot/camera_reset_fixed.py"
        ], capture_output=True, text=True)

        print("Camera GPIO reset output:")
        print(result.stdout)
        if result.stderr:
            print("Errors:")
            print(result.stderr)

    except Exception as e:
        print(f"GPIO reset failed: {e}")

def start_camera_service():
    """Start hobot camera service"""
    try:
        # Kill any existing cam-service processes
        subprocess.run(["sudo", "pkill", "-f", "cam-service"], capture_output=True)
        time.sleep(1)

        # Start camera service
        result = subprocess.run([
            "sudo", "/usr/hobot/bin/cam-service",
            "-s", "4,2,4,2", "-i", "6", "-V", "6"
        ], capture_output=True, text=True, timeout=10)

        print("Camera service output:")
        print(result.stdout)
        if result.stderr:
            print("Service errors:")
            print(result.stderr)

        return result.returncode == 0

    except subprocess.TimeoutExpired:
        print("Camera service started (timeout expected for daemon)")
        return True
    except Exception as e:
        print(f"Camera service failed: {e}")
        return False

def check_video_devices():
    """Check for video devices"""
    try:
        result = subprocess.run(["ls", "/dev/video*"],
                              shell=True, capture_output=True, text=True)
        if result.returncode == 0:
            print("✓ Video devices found:")
            print(result.stdout)
            return True
        else:
            print("✗ No video devices found")
            return False
    except Exception as e:
        print(f"Video device check failed: {e}")
        return False

def check_i2c_devices():
    """Check I2C for camera devices"""
    try:
        for bus in [0, 1, 2, 3, 4, 5, 6, 7]:
            result = subprocess.run([
                "sudo", "i2cdetect", "-y", str(bus)
            ], capture_output=True, text=True)

            if "10" in result.stdout or "42" in result.stdout:
                print(f"✓ I2C device found on bus {bus}")
                print(result.stdout)
                return True

        print("✗ No I2C camera devices found")
        return False

    except Exception as e:
        print(f"I2C check failed: {e}")
        return False

def main():
    """Main initialization sequence"""
    print("=== XleRobot IMX219 Camera Initialization ===")

    print("\n1. Loading kernel modules...")
    load_kernel_modules()

    print("\n2. Resetting camera GPIO...")
    reset_camera_gpio()

    print("\n3. Checking I2C devices...")
    i2c_ok = check_i2c_devices()

    print("\n4. Starting camera service...")
    service_ok = start_camera_service()

    print("\n5. Checking video devices...")
    video_ok = check_video_devices()

    print("\n=== Initialization Summary ===")
    print(f"I2C Devices: {'✓' if i2c_ok else '✗'}")
    print(f"Camera Service: {'✓' if service_ok else '✗'}")
    print(f"Video Devices: {'✓' if video_ok else '✗'}")

    if i2c_ok and service_ok:
        print("\n✓ Camera initialization appears successful!")

        # Wait a moment and check again
        time.sleep(2)
        check_video_devices()

        return 0
    else:
        print("\n✗ Camera initialization failed")
        return 1

if __name__ == "__main__":
    sys.exit(main())