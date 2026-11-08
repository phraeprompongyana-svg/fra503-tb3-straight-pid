import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'tb3_straight_tune'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Phraeprompong Yanawin',
    maintainer_email='phraeprompong.yana@mail.kmutt.ac.th',
    description='Straight-line driving for TurtleBot3 Burger with heading PID, step test and lab run planner',
    license='MIT',
    entry_points={
        'console_scripts': [
            'run_straight_tuned = tb3_straight_tune.run_straight_tuned:main',
            'summarize_tune = tb3_straight_tune.summarize_tune:main',
            'tune_sweep = tb3_straight_tune.tune_sweep:main',
            'lab_plan = tb3_straight_tune.lab_plan:main',
        ],
    },
)
