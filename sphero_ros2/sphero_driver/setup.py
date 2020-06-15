from setuptools import setup

package_name = 'sphero_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    scripts=['scripts/test.py'],
    install_requires=['setuptools', 'bluez'],
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
)
