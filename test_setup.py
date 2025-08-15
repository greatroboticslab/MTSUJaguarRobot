#!/usr/bin/env python3
"""
Test script to verify the robot control system setup
Run this after installation to check if everything is working correctly
"""

import sys
import importlib
import subprocess
import socket
import time

def test_python_version():
    """Test if Python version is 3.10 or higher"""
    print("Testing Python version...")
    version = sys.version_info
    if version.major >= 3 and version.minor >= 10:
        print(f"✅ Python {version.major}.{version.minor}.{version.micro} is supported")
        return True
    else:
        print(f"❌ Python {version.major}.{version.minor}.{version.micro} is too old. Need 3.10+")
        return False

def test_dependencies():
    """Test if all required packages are installed"""
    print("\nTesting dependencies...")
    dependencies = ['flask', 'paho.mqtt.client', 'cv2']
    
    for dep in dependencies:
        try:
            if dep == 'paho.mqtt.client':
                importlib.import_module('paho.mqtt.client')
                print(f"✅ paho-mqtt is installed")
            elif dep == 'cv2':
                importlib.import_module('cv2')
                print(f"✅ opencv-python is installed")
            else:
                importlib.import_module(dep)
                print(f"✅ {dep} is installed")
        except ImportError:
            print(f"❌ {dep} is not installed. Run: pip install -r requirements.txt")
            return False
    return True

def test_app_import():
    """Test if the main app can be imported without errors"""
    print("\nTesting app import...")
    try:
        # Change to directory for import
        import os
        os.chdir('/Users/danielpowers/Desktop/MTSUJaguarRobot')
        
        # Try importing the main modules
        from flask import Flask
        print("✅ Flask import successful")
        
        import paho.mqtt.client as mqtt
        print("✅ MQTT import successful")
        
        import cv2
        print("✅ OpenCV import successful")
        
        return True
    except Exception as e:
        print(f"❌ Import error: {e}")
        return False

def test_port_availability():
    """Test if port 5014 is available"""
    print("\nTesting port availability...")
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(1)
        result = sock.connect_ex(('127.0.0.1', 5014))
        sock.close()
        
        if result == 0:
            print("⚠️  Port 5014 is already in use. You may need to kill existing process:")
            print("   lsof -i :5014")
            print("   kill -9 <PID>")
            return False
        else:
            print("✅ Port 5014 is available")
            return True
    except Exception as e:
        print(f"❌ Port test error: {e}")
        return False

def test_file_structure():
    """Test if all required files exist"""
    print("\nTesting file structure...")
    import os
    
    required_files = [
        'app.py',
        'network_config.py', 
        'requirements.txt',
        'templates/index.html',
        'templates/camera.html',
        'static/robot.png',
        'static/myhouse.osm'
    ]
    
    missing_files = []
    for file_path in required_files:
        if not os.path.exists(file_path):
            missing_files.append(file_path)
        else:
            print(f"✅ {file_path} exists")
    
    if missing_files:
        print(f"❌ Missing files: {missing_files}")
        return False
    
    return True

def run_all_tests():
    """Run all tests and provide summary"""
    print("🚀 Robot Control System Setup Test")
    print("=" * 40)
    
    tests = [
        ("Python Version", test_python_version),
        ("Dependencies", test_dependencies),
        ("App Import", test_app_import),
        ("Port Availability", test_port_availability),
        ("File Structure", test_file_structure)
    ]
    
    results = []
    for test_name, test_func in tests:
        try:
            result = test_func()
            results.append((test_name, result))
        except Exception as e:
            print(f"❌ {test_name} failed with exception: {e}")
            results.append((test_name, False))
    
    print("\n" + "=" * 40)
    print("📊 TEST SUMMARY")
    print("=" * 40)
    
    passed = 0
    total = len(results)
    
    for test_name, result in results:
        status = "✅ PASS" if result else "❌ FAIL"
        print(f"{test_name:.<20} {status}")
        if result:
            passed += 1
    
    print(f"\nResults: {passed}/{total} tests passed")
    
    if passed == total:
        print("\n🎉 ALL TESTS PASSED!")
        print("Your setup is ready. You can now run:")
        print("   python3 app.py")
        print("Then open: http://127.0.0.1:5014")
    else:
        print("\n⚠️  Some tests failed. Please fix the issues above.")
        print("See the README.md 'Getting Started' section for help.")
    
    return passed == total

if __name__ == "__main__":
    run_all_tests()
