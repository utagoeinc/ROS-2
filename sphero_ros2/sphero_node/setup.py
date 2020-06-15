from setuptools import setup

package_name = 'sphero_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    py_modules=[
        'sphero'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Allison Thackston',
    author_email='allison@lyonthackston.com',
    maintainer='Allison Thackston',
    maintainer_email='allison@lyonthackston.com',
    keywords=['ROS2'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Sphero Python driver',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'sphero = sphero:main',
    ],
},
)
