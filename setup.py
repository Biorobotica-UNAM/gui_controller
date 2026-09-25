from setuptools import find_packages, setup

package_name = 'gui_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    # Incluye todos los archivos .ui dentro del paquete Python
    package_data={
        package_name: ['*.ui', '*.yaml', '*.yml'],
    },
    include_package_data=True,
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='sergio',
    maintainer_email='sergio081096@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        'gui_controller = gui_controller.gui_controller:main'
        ],
    },
)
