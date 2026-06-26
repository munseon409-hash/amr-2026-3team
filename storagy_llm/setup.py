from setuptools import find_packages, setup
from glob import glob

package_name = 'storagy_llm'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['storagy_llm/.env']),
        ('share/' + package_name + '/params',[
            'params/prompt.yaml',
            'params/points.yaml',
        ]),
        ('share/' + package_name + '/web', glob('web/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bluephysi01',
    maintainer_email='bluephysi01@todo.todo',
    description='LLM control and vision explanation package for Storagy simulation',
    license='Apache 2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'agent_service = storagy_llm.agent_service:main',
            'agent_client = storagy_llm.agent_client:main',
            'web_dashboard = storagy_llm.web_dashboard:main',
        ],
    },
)
