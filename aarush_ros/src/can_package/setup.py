from setuptools import find_packages, setup

package_name = 'can_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jaay',
    maintainer_email='jaayanth.jr@gmail.com',
    description='CAN Package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'can_rx_node = can_package.can_rx_node_v3:main',
            'can_node_sim = can_package.can_node_sim:main',
            'can_tx_node = can_package.can_tx_node:main',
            'can_rx_node_dup = can_package.can_rx_node_dup:main',
            'can_rx_node_test = can_package.can_rx_node_test:main',

        ],
    },
)
