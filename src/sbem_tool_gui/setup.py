from setuptools import setup

package_name = 'sbem_tool_gui'

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
    maintainer='morolinux',
    maintainer_email='enrimoro2003@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['create_gui = sbem_tool_gui.create_gui:main',
                            'gui_node = sbem_tool_gui.gui_node:main'
                            
        ],
    },
)
