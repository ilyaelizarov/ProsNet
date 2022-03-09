# ProsNet
ProsNet is a modelica library for modeling prosumer-based heat networks.
A detailed description on the motivation, design and validation of this library can be found in the paper [ProsNet – a Modelica library for prosumer-based heat networks: description and validation](https://www.doi.org/10.1088/1742-6596/2042/1/012031).

The library is free and open-source, and its models inherit from the Modelica Standard Library and the IBPSA library. Necessary components of the IBPSA library were copied to ProsNet so the library can be used with the default workspace available for every simulation environment. All fluid components are initialized with the incompressible water media model. Compatibility with the Modelica Standard Library and the IBPSA library is assured.
The library was developed and tested within the Dymola Simulation Environment.

# Overview
A prosumer can operate either in production or consumption mode with respect to a heat network. It can also be detached or attached to the network by setting appropriate participation: participation or non-participation. Control elements on the primary side (network) are a control valve and a feed-in pump. Together with a heat exchanger, the control elements allow bidirectional heat transfer between the network and technology-dependent heat supply/demand on the secondary side depending on the temperature gradient between the sides. The prosumer model and other components like pipes and sensors can be simulated for steady-state (statically) or transient-state (dynamically).

![ProsScheme](https://user-images.githubusercontent.com/54630145/157418834-7e4c67e4-211d-4cb5-a4be-304b689c7e67.gif)

There are four inputs that determine its interaction with the network:
* Operation mode <img src="https://render.githubusercontent.com/render/math?math=\mu">: consumption, production
* Participation or non-participation in heat exchange with the network <img src="https://render.githubusercontent.com/render/math?math=\pi">
* Non-dimensional flow coefficient for the control valve <img src="https://render.githubusercontent.com/render/math?math=\kappa">
* Feed-in pump velocity <img src="https://render.githubusercontent.com/render/math?math=u">

Operating mode and participation have integer counterparts, which are used as inputs of the multiple models: 
<img src="https://render.githubusercontent.com/render/math?math=\mu=1"> for production mode, <img src="https://render.githubusercontent.com/render/math?math=\mu=-1"> for consumption mode, <img src="https://render.githubusercontent.com/render/math?math=\pi=1"> when participating in heat exchage is on, <img src="https://render.githubusercontent.com/render/math?math=\pi=0"> for non-participating. Mode details can be found in the ProsumerIdeal model.

A Prosumers package contains prosumer models for different heat generation and consumption technology. Examples of prosumer-based heat networks are given in the Examples package. Additional insights on how the models work can be gained from the validation packages. The validation models allow confirming the expected behavior of models.

# Requirements
The library is developed and tested with the modeling and simulation environment "Dymola – Dynamic Modeling Laboratory" by Dessault Systèmes. The used version of Dymola is "Version 2021x (64-bit), 2020-11-26". The used version of [Modelica](https://github.com/modelica/ModelicaStandardLibrary) is 4.0.0. The library is using parts of the [IBPSA library](https://github.com/ibpsa/modelica-ibpsa) in version 3.0.0.
