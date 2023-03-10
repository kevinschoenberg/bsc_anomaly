from setuptools import setup

package_name = 'bsc_anomaly'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kevinschonberg',
    maintainer_email='kevinschonberg@todo.todo',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'LSDNode = bsc_anomaly.LSDNode:main',
        	'LoggerNode = bsc_anomaly.LoggerNode:main',
            'KeyNode = bsc_anomaly.KeyNode:main'
        ],
    },
)
