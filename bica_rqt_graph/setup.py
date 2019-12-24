from setuptools import setup

package_name = 'bica_rqt_graph'
setup(
    name=package_name,
    version='0.2.0',
    package_dir={'': 'src'},
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name + '/resource', ['resource/BicaGraph.ui']),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['plugin.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Francisco Martin',
    maintainer='Francisco Martin',
    maintainer_email='fmrico@gmail.com',
    keywords=['BICA'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: BSD License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description=(
        'bica_rqt_graph provides a GUI plugin for visualizing the BICA graph.'
    ),
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bica_rqt_graph = bica_rqt_graph.main:main',
        ],
    },
)
