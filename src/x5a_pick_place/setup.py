from setuptools import setup
from glob import glob
package_name='x5a_pick_place'
setup(
  name=package_name,
  version='0.1.0',
  packages=[package_name],
  data_files=[
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/config', glob('config/*')),
    ('share/' + package_name + '/launch', glob('launch/*.py')),
  ],
  install_requires=['setuptools'],
  zip_safe=True,
  maintainer='x5a user',
  maintainer_email='user@example.com',
  description='X5A fixed-pose pick place',
  license='BSD',
  entry_points={'console_scripts': [
    'pick_place_node = x5a_pick_place.pick_place_node:main',
  ]},
)
