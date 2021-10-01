from setuptools import setup

package_name = 'flir_lepton_purethermal2'

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
    maintainer='hungtc',
    maintainer_email='hungtc@solab.me.ntu.edu.tw',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'purethermal2_node = flir_lepton_purethermal2.purethermal2_node:main',
            'purethermal2_subscriber = flir_lepton_purethermal2.purethermal2_subscriber:main'
        ],
    },
)
