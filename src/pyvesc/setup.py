from setuptools import setup

package_name = 'pyvesc'
VERSION = '1.0.5'

setup(
  name='pyvesc',
  packages=[package_name],
  version=VERSION,
  description='Python implementation of the VESC communication protocol.',
  author='Liam Bindle',
  author_email='liambindle@gmail.com',
  url='https://github.com/LiamBindle/PyVESC',
  download_url='https://github.com/LiamBindle/PyVESC/tarball/' + VERSION,
  keywords=['vesc', 'VESC', 'communication', 'protocol', 'packet'],
  classifiers=[],
  install_requires=['crccheck'],
  data_files=[
      ('share/ament_index/resource_index/packages',
          ['resource/' + package_name]),
      ('share/' + package_name, ['package.xml']),
  ],
)
