import os
from glob import glob
from setuptools import setup

package_name = 'mobile_robot'

setup(
    name=package_name,
    version='0.0.1',
    # Especifica explícitamente el paquete a incluir.
    # Python buscará una carpeta con el mismo nombre que 'package_name' que contenga un __init__.py.
    packages=[package_name],
    # Archivos adicionales que deben ser instalados.
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # El archivo package.xml es obligatorio.
        ('share/' + package_name, ['package.xml']),
        # Aquí incluimos de forma robusta todos los archivos de lanzamiento (.py)
        # que se encuentren en la carpeta 'launch'.
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tu_nombre',
    maintainer_email='tu_email@example.com',
    description='Paquete de control para robot móvil con ESP32',
    license='Apache 2.0',
    tests_require=['pytest'],
    # Define los 'puntos de entrada' para tus scripts de Python.
    # Esto permite ejecutar los nodos con 'ros2 run'.
    entry_points={
        'console_scripts': [
            # Formato: 'nombre_ejecutable = nombre_paquete.nombre_archivo_python:funcion_a_llamar'
            'wifi_bridge = mobile_robot.wifi_bridge:main',
            'obstacle_avoider = mobile_robot.obstacle_avoider:main',
        ],
    },
)
