# Navigate to the directory containing 'check.py' and execute the Python script
python check.py lbuild

# Change to the project directory
Set-Location -Path "aruw-mcb-project"

# Run the SCons build command with the specified robot
scons ozone robot=SENTRY_HYDRA

# Return to the parent directory
Set-Location -Path ..