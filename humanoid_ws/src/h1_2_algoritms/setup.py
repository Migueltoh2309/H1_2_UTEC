from setuptools import find_packages, setup

package_name = 'h1_2_algoritms'

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
    maintainer='utec',
    maintainer_email='molortegui@utec.edu.pe',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'fk_whole_body_h1_2 = h1_2_algoritms.fk_whole_body_h1_2:main',
            'ik_whole_body_h1_2 = h1_2_algoritms.ik_whole_body_h1_2:main',
            'telekeyop_whole_body = h1_2_algoritms.telekeyop_whole_body:main',
            'null_control_whole_body = h1_2_algoritms.null_control_whole_body:main',
            'kine_control_whole_body = h1_2_algoritms.kine_control_whole_body:main',
            'QP_whole_body = h1_2_algoritms.QP_whole_body:main',
            #'metodo_parte1 = h1_2_algoritms.metodo_parte1:main',
            #'metodo_parte2 = h1_2_algoritms.metodo_parte2:main',
            #'metodo_parte3 = h1_2_algoritms.metodo_parte3:main',
        ],
    },
)
