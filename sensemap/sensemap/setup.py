Sure, here's the content for the file `/sensemap/setup.py`:

from setuptools import setup, find_packages

setup(
    name='sensemap',
    version='0.1.0',
    author='Your Name',
    author_email='your.email@example.com',
    description='Indoor semantic mapping using visual SLAM and object detection',
    packages=find_packages(where='src'),
    package_dir={'': 'src'},
    install_requires=[
        'opencv-python',
        'torch',
        'torchvision',
        'pcl',
        'numpy',
        'matplotlib',
        'plotly',
        'rospy',
        'sensor_msgs',
        'geometry_msgs',
        'cv_bridge',
    ],
    entry_points={
        'console_scripts': [
            'sensemap=sensemap.main:main',  # Adjust the entry point as needed
        ],
    },
)