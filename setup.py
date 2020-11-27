from glob import glob

from setuptools import setup

package_name = 'planning_simulator_launcher'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*')),
    ],
    install_requires=[
        'launch',
        'launch_ros',
        'launch_xml',
        'numpy',
        'pyyaml',
        'setuptools',
        'termcolor',
    ],
    zip_safe=True,
    author='Masaya Kataoka, Tomoya Kimura, Tatsuya Yamasaki',
    author_email='masaya.kataoka@tier4.jp, tomoya.kimura@tier4.jp, httperror@404-notfound.jp',
    maintainer='Masaya Kataoka, Tomoya Kimura, Tatsuya Yamasaki',
    maintainer_email='masaya.kataoka@tier4.jp, tomoya.kimura@tier4.jp, httperror@404-notfound.jp',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='The planning_simulator_launcher package.',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'launch_main = planning_simulator_launcher.launch:main',
            'show_result_main = planning_simulator_launcher.show_result:main',
        ],
    },
)
