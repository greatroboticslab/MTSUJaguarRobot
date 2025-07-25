#!/usr/bin/env python3

# sets up network configuration to run robot
# run `python3 network_config.py robot` to switch to robot network
# run `python3 network_config.py internet` to switch to internet network
# you'll still have to manually connect to the robot's Wi-Fi network
"""
Network configuration switcher for robot development
Run this script to quickly switch between robot network and internet network
"""

import subprocess
import sys

def run_command(cmd):
    """Run a shell command and return the result"""
    try:
        result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
        return result.returncode == 0, result.stdout, result.stderr
    except Exception as e:
        return False, "", str(e)

def get_network_service():
    """Get the primary network service name"""
    success, output, error = run_command("networksetup -listallnetworkservices | grep -E '(Wi-Fi|Ethernet)' | head -1")
    if success and output.strip():
        return output.strip()
    return "Wi-Fi"  # Default fallback

def set_robot_network():
    """Configure network for robot communication"""
    service = get_network_service()
    print(f"Setting up robot network on {service}...")
    
    commands = [
        f"sudo networksetup -setmanual '{service}' 192.168.0.109 255.255.255.0 192.168.0.1",
        f"sudo networksetup -setdnsservers '{service}' 192.168.0.1"
    ]
    
    for cmd in commands:
        success, stdout, stderr = run_command(cmd)
        if not success:
            print(f"Error: {stderr}")
            return False
    
    print("✅ Robot network configured!")
    print("   IP: 192.168.0.109")
    print("   You can now connect to robot (192.168.0.60) and camera (192.168.0.65)")
    return True

def set_dhcp_network():
    """Configure network for internet access (DHCP)"""
    service = get_network_service()
    print(f"Setting up DHCP network on {service}...")
    
    commands = [
        f"sudo networksetup -setdhcp '{service}'",
        f"sudo networksetup -setdnsservers '{service}' 8.8.8.8 8.8.4.4"
    ]
    
    for cmd in commands:
        success, stdout, stderr = run_command(cmd)
        if not success:
            print(f"Error: {stderr}")
            return False
    
    print("✅ DHCP network configured!")
    print("   You now have internet access for debugging")
    return True

def show_current_config():
    """Show current network configuration"""
    service = get_network_service()
    print(f"\nCurrent network configuration for {service}:")
    
    success, output, error = run_command(f"networksetup -getinfo '{service}'")
    if success:
        print(output)
    else:
        print(f"Error getting network info: {error}")

def main():
    if len(sys.argv) != 2 or sys.argv[1] not in ['robot', 'internet', 'status']:
        print("Usage:")
        print("  python3 network_switch.py robot    - Switch to robot network (192.168.0.109)")
        print("  python3 network_switch.py internet - Switch to internet network (DHCP)")
        print("  python3 network_switch.py status   - Show current network status")
        sys.exit(1)
    
    mode = sys.argv[1]
    
    if mode == 'robot':
        set_robot_network()
    elif mode == 'internet':
        set_dhcp_network()
    elif mode == 'status':
        show_current_config()
    
    print("\nNote: It may take a few seconds for the network change to take effect.")

if __name__ == "__main__":
    main()
