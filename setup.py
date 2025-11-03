# Copyright 2025 Marc Duclusaud & Grégoire Passault

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at:

#     http://www.apache.org/licenses/LICENSE-2.0

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
    install_requires=["numpy", "colorama"],
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
    },
    include_package_data=True,
    package_data={},
    python_requires=">=3.6",
)
