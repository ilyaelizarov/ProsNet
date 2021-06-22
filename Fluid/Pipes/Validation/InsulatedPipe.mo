within ProsNet.Fluid.Pipes.Validation;
model InsulatedPipe "Tests InsulatedPipe model"

  extends Modelica.Icons.Example;
  .ProsNet.Fluid.Pipes.InsulatedPipe insPipe(T_amb=253.15, length=100)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
  Sources.Boundary_pT bou(p=200000, nPorts=1)
    annotation (Placement(transformation(extent={{-80,-10},{-60,10}})));
  Sources.Boundary_pT bou1(p=100000, nPorts=1)
    annotation (Placement(transformation(extent={{80,-10},{60,10}})));
  Sensors.TemperatureTwoPort senTem(m_flow_nominal=1, initType=Modelica.Blocks.Types.Init.SteadyState)
    annotation (Placement(transformation(extent={{-42,-10},{-22,10}})));
  Sensors.TemperatureTwoPort senTem1(m_flow_nominal=1, initType=Modelica.Blocks.Types.Init.SteadyState)
    annotation (Placement(transformation(extent={{24,-10},{44,10}})));
  .ProsNet.Fluid.Pipes.InsulatedPipe insPipe1 annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={42,-38})));
  Sources.Boundary_pT bou2(p=500000, nPorts=1) annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={42,-78})));
equation
  connect(bou.ports[1], senTem.port_a)
    annotation (Line(points={{-60,0},{-42,0}}, color={0,127,255}));
  connect(senTem.port_b, insPipe.port_a)
    annotation (Line(points={{-22,0},{-10,0}}, color={0,127,255}));
  connect(insPipe.port_b, senTem1.port_a)
    annotation (Line(points={{10,0},{24,0}}, color={0,127,255}));
  connect(senTem1.port_b, bou1.ports[1])
    annotation (Line(points={{44,0},{60,0}}, color={0,127,255}));
  connect(senTem1.port_b, insPipe1.port_a)
    annotation (Line(points={{44,0},{44,-28},{42,-28}}, color={0,127,255}));
  connect(insPipe1.port_b, bou2.ports[1])
    annotation (Line(points={{42,-48},{42,-68}}, color={0,127,255}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    Documentation(info= "<html><head></head><body><p>This validation model tests the thermal and minor hydraulic losses feature of the main model.</p>
</body></html>"));
end InsulatedPipe;