#!/usr/bin/env python3
import os
import shutil

dir_path = os.path.dirname(os.path.abspath(__file__))
backup_path = os.path.join(dir_path, 'custom_road_backup.sdf')
target_path = os.path.join(dir_path, 'custom_road.sdf')

# 1. Start with the clean backup
if not os.path.exists(backup_path):
    # If backup doesn't exist, create it from target
    shutil.copyfile(target_path, backup_path)
else:
    shutil.copyfile(backup_path, target_path)

# 2. Check environment variable
traffic = os.environ.get('TRAFFIC', '').lower()
if traffic in ('true', '1', 'yes', 'on'):
    print("Obstacles/Traffic enabled (static boxes removed).")
else:
    print("Obstacles/Traffic disabled. custom_road.sdf is clean.")
