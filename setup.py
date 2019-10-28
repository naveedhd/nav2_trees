from setuptools import setup

package_name = 'nav2_trees'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Naveed Usmani',
    author_email='naveedhd@gmail.com',
    maintainer='Naveed Usmani',
    maintainer_email='naveedhd@gmail.com',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='nav2 trees using py_trees.',
    license='Apache License, Version 2.0',
    entry_points={
        'console_scripts': [
            'simple = ' + package_name + '.simple:main',
        ],
    },
)