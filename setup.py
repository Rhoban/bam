import setuptools

setuptools.setup(
    name="better-actuator-model",
    version="0.0.1",
    author="Rhoban team",
    author_email="team@rhoban.com",
    description="Work in progress",
    long_description="",
    long_description_content_type="text/markdown",
    url="https://github.com/rhoban/bam/",
    packages=setuptools.find_packages(),
    entry_points={},
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    keywords="robot robotics",
    install_requires=["numpy"],
    extras_require={
        "dev": [
            "zmq",
            "protobuf==3.20",
            "dynamixel_sdk",
            "pyserial",
            "optuna",
            "wandb",
            "pygame"
        ],
        "feetech": [
            "feetech-servo-sdk==1.0.0",
            "tqdm==4.67.1"
        ]
    },
    include_package_data=True,
    package_data={},
    python_requires=">=3.6",
)
