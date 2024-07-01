within ProsNet.Prosumers;
model MF5_indep "Example model of a building with loads provided as time series and
  connected to an ETS for cooling"
  extends Modelica.Icons.Example;
  package Medium=ProsNet.Fluid.Building_Fluid.Utili.Media.Water
    "Medium model";
  Modelica.Blocks.Sources.RealExpression realExpression(y=1)
    annotation (Placement(transformation(extent={{-504,-4},{-484,16}})));
  Modelica.Blocks.Sources.RealExpression realExpression1(y=1)
    annotation (Placement(transformation(extent={{-436,-336},{-416,-316}})));
  Modelica.Blocks.Sources.Ramp y(
    height=1,
    duration=1,
    offset=0) "Control signal"
    annotation (Placement(transformation(extent={{-290,56},{-270,76}})));

  Modelica.Blocks.Sources.CombiTimeTable sch(
    tableOnFile=true,
    tableName="tab1",
    fileName=Modelica.Utilities.Files.loadResource(
        "modelica://ProsNet/Data/Loads/HotWater/DHW_ApartmentMidRise.mos"),
    smoothness=Modelica.Blocks.Types.Smoothness.ContinuousDerivative,
    extrapolation=Modelica.Blocks.Types.Extrapolation.Periodic)
    "Domestic hot water fixture draw fraction schedule"
    annotation (Placement(transformation(extent={{-9,-9},{9,9}},
        rotation=180,
        origin={393,-179})));

  Modelica.Blocks.Sources.Constant conTSetMix(k(
      final unit="K",
      displayUnit="degC") = 308.15)
    "Temperature setpoint for mixed water supply to fixture"
    annotation (Placement(transformation(extent={{-8,-8},{8,8}},
        rotation=180,
        origin={392,-152})));

  Modelica.Blocks.Sources.Ramp y1(
    height=1,
    duration=1,
    offset=0) "Control signal"
    annotation (Placement(transformation(extent={{92,18},{112,38}})));

  Modelica.Blocks.Math.Gain gaiHea(k=1E6) "Gain for heating"
    annotation (Placement(visible=true,transformation(origin={434,44},
                                                                     extent={{-6,-6},{6,6}},rotation=0)));
  Modelica.Blocks.Sources.Constant TSetHea(k=273.15 + 20)
    "Set-point for heating"
    annotation (Placement(visible=true, transformation(origin={388,44},extent={{-6,-6},{6,6}},rotation=0)));
  Modelica.Blocks.Sources.Constant TSetCoo(k=273.15 + 27)
    "Set-point for cooling"
    annotation (Placement(visible=true, transformation(origin={388,18},extent={{-6,-6},{6,6}},rotation=0)));
  Modelica.Blocks.Math.Gain gaiCoo(k=-1E6) "Gain for cooling"
    annotation (Placement(visible=true,transformation(origin={434,18},
                                                                     extent={{-6,-6},{6,6}},rotation=0)));

  Components.Generators.Laboratory_Models.PVSimpleOriented pv(
    A=200e3/800/0.12,
    til=0.34906585039887,
    azi=-0.78539816339745,
    V_nominal=480)
    annotation (Placement(transformation(extent={{476,-370},{564,-278}})));

  Components.Consumers.ThermostaticMixingValve theMixVal(mMix_flow_nominal=1.2*
        datWatHea.mDom_flow_nominal)
    annotation (Placement(transformation(extent={{254,-242},{294,-206}})));
  Components.Electrical.Inductive acLoad1(linearized=false, mode=ProsNet.Fluid.Building_Fluid.Utili.Electrical.Types.Load.VariableZ_P_input)
    annotation (Placement(transformation(
        extent={{-24,-16},{24,16}},
        rotation=90,
        origin={168,-446})));
  Components.Electrical.Inductive acLoad(linearized=false, mode=ProsNet.Fluid.Building_Fluid.Utili.Electrical.Types.Load.VariableZ_P_input)
    annotation (Placement(transformation(
        extent={{-24,-16},{24,16}},
        rotation=90,
        origin={302,-442})));
  Components.Electrical.ACDCConverter conv(conversionFactor=480/480, eta=0.9)
    annotation (Placement(transformation(extent={{-72,-644},{16,-572}})));
  Components.Electrical.BatteryControl con
    annotation (Placement(transformation(extent={{-230,-644},{-198,-612}})));
  Components.Electrical.Battery bat(EMax=500e3*4*3600, V_nominal=480)
    annotation (Placement(transformation(extent={{-470,-652},{-374,-568}})));
  Components.Electrical.Grid gri(
    f=60,
    V=480,
    phiSou=0)
    annotation (Placement(transformation(extent={{12,-530},{88,-458}})));
  Components.Consumers.GenericDomesticHotWaterWithHeatExchanger datWatHea(VTan=0.1892706,
      mDom_flow_nominal=6.52944E-06*1000)
    annotation (Placement(transformation(extent={{384,-232},{404,-212}})));
  Components.Valves.ThreeWayLinear val3(
  redeclare package Medium = ProsNet.Fluid.Building_Fluid.Utili.Media.Water,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
    use_inputFilter=false,
    m_flow_nominal=2,
    dpValve_nominal=1,
    l={0.05,0.05})
    annotation (Placement(transformation(extent={{-230,-30},{-210,-10}})));
  Components.Valves.ThreeWayLinear val4(
    redeclare package Medium = Fluid.Building_Fluid.Utili.Media.Water,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
    use_inputFilter=false,
    m_flow_nominal=2,
    dpValve_nominal=1,
    l={0.05,0.05})
    annotation (Placement(transformation(extent={{-162,-150},{-142,-130}})));
  Components.Valves.ThreeWayLinear val1(
    redeclare package Medium = Fluid.Building_Fluid.Utili.Media.Water,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
    use_inputFilter=false,
    m_flow_nominal=2,
    dpValve_nominal=1,
    l={0.05,0.05})
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=180,
        origin={156,-142})));
  Components.BoundryCondition.ReaderTMY3 weaDat(filNam=
        ModelicaServices.ExternalReferences.loadResource("modelica://ProsNet/Data/weatherdata/USA_CA_San.Francisco.Intl.AP.724940_TMY3.mos"))
    annotation (Placement(transformation(extent={{-68,120},{-48,140}})));
  Components.Storage.Stratified tan(
    redeclare package Medium = ProsNet.Fluid.Building_Fluid.Utili.Media.Water,
    m_flow_nominal=2,
    show_T=true,
    VTan(displayUnit="l") = 0.775,
    hTan=2,
    dIns=0.3,
    kIns=0.2,
    nSeg=10,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
    T_start=353.15)
    annotation (Placement(transformation(extent={{-104,-62},{14,56}})));
  Components.Consumers.SingleZoneFloor sinZonFlo(redeclare package Medium =
        Modelica.Media.Water.ConstantPropertyLiquidWater, use_windPressure=
        false)
    annotation (Placement(transformation(extent={{204,-26},{244,14}})));
  Components.Consumers.BuildingTimeSeries buiHea(
    have_chiWat=false,
    filNam="modelica://ProsNet/SwissResidential_20190916.mos",
    nPorts_aHeaWat=2,
    nPorts_bHeaWat=2)
    annotation (Placement(transformation(extent={{172,-48},{278,64}})));
  Controls.LimPID conHeaPID1(
    k=0.1,
    Ti=300,
    reverseActing=true)
    annotation (Placement(transformation(extent={{402,36},{416,50}})));
  Controls.LimPID conCooPID(
    k=0.1,
    Ti=300,
    reverseActing=true)
    annotation (Placement(transformation(extent={{402,8},{418,24}})));
  Modelica.Blocks.Sources.BooleanExpression booleanExpression(y=true)
    annotation (Placement(transformation(extent={{-698,-330},{-676,-308}})));
  Controls.GeneratorControlPackage.ControlGenerator.GeneratorControl NeoTowerControl(TStartCHP
      =333, TStopCHP=343)
    annotation (Placement(transformation(extent={{-612,-354},{-520,-262}})));
  Controls.GeneratorControlPackage.ControlGenerator.GeneratorControl ChpControl(TStartCHP
      =338, TStopCHP=343)
    annotation (Placement(transformation(extent={{-640,0},{-556,84}})));
  Components.Generators.Digital_Twins.WolfCGB50_GC wolfCGB50_GC1
    annotation (Placement(transformation(extent={{-458,-12},{-374,72}})));
  Components.Generators.Digital_Twins.NeoTower5_GC neoTower5_GC1
    annotation (Placement(transformation(extent={{-400,-358},{-296,-254}})));
  Modelica.Blocks.Sources.Constant conTSetHot(k(
      final unit="K",
      displayUnit="degC") = 313.15)
    "Temperature setpoint for hot water supply to fixture"
    annotation (Placement(transformation(extent={{54,-352},{68,-338}})));
  Components.Consumers.DHW domHotWatTan(
    redeclare package MediumDom = Fluid.Building_Fluid.Utili.Media.Water,
    redeclare package MediumHea = Fluid.Building_Fluid.Utili.Media.Water,
    dat=datWatHea)
    annotation (Placement(transformation(extent={{100,-380},{172,-308}})));
  Fluid.Sources.Boundary_pT souCol1(redeclare package Medium = Media.Water,
      nPorts=2)
    annotation (Placement(transformation(extent={{40,-296},{60,-276}})));
  Components.Valves.ThreeWayLinear val2(
    redeclare package Medium = Fluid.Building_Fluid.Utili.Media.Water,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
    use_inputFilter=false,
    m_flow_nominal=2,
    dpValve_nominal=1,
    l={0.05,0.05})
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=0,
        origin={80,-24})));
  Modelica.Blocks.Sources.BooleanExpression booleanExpression1(y=true)
    annotation (Placement(transformation(extent={{-714,22},{-692,44}})));
equation

  connect(theMixVal.TMixSet, conTSetMix.y) annotation (Line(points={{252,-220.4},
          {244,-220.4},{244,-192},{372,-192},{372,-152},{383.2,-152}},
                  color={0,0,127}));
  connect(sch.y[1], theMixVal.yMixSet) annotation (Line(points={{383.1,-179},{
          240,-179},{240,-209.6},{252,-209.6}},
                                            color={0,0,127}));
  connect(pv.terminal, conv.terminal_p) annotation (Line(points={{476,-324},{464,
          -324},{464,-608},{16,-608}}, color={0,0,255}));
  connect(bat.terminal, conv.terminal_p) annotation (Line(points={{-470,-610},{-488,
          -610},{-488,-664},{28,-664},{28,-608},{16,-608}}, color={0,0,255}));
  connect(con.SOC, bat.SOC) annotation (Line(points={{-232,-628},{-348,-628},{-348,
          -584.8},{-369.2,-584.8}}, color={0,0,127}));
  connect(con.y, bat.P) annotation (Line(points={{-197,-628},{-188,-628},{-188,-540},
          {-422,-540},{-422,-568}}, color={0,0,127}));
  connect(acLoad.terminal, gri.terminal) annotation (Line(points={{302,-466},{302,
          -532},{100,-532},{100,-530},{50,-530}}, color={0,120,120}));
  connect(acLoad1.terminal, gri.terminal) annotation (Line(points={{168,-470},{168,
          -532},{110,-532},{110,-530},{50,-530}}, color={0,120,120}));
  connect(conv.terminal_n, gri.terminal) annotation (Line(points={{-72,-608},{-84,
          -608},{-84,-530},{50,-530}}, color={0,120,120}));
  connect(val3.y, y.y)
    annotation (Line(points={{-220,-8},{-220,66},{-269,66}}, color={0,0,127}));
  connect(val4.y, y.y) annotation (Line(points={{-152,-128},{-152,66},{-269,66}},
        color={0,0,127}));
  connect(val1.y, y1.y) annotation (Line(points={{156,-154},{156,-164},{120,-164},
          {120,28},{113,28}},                    color={0,0,127}));
  connect(pv.weaBus, weaDat.weaBus) annotation (Line(
      points={{520,-282.6},{520,130},{-48,130}},
      color={255,204,51},
      thickness=0.5));

  connect(sinZonFlo.weaBus, weaDat.weaBus) annotation (Line(
      points={{210.8,11},{132,11},{132,130},{-48,130}},
      color={255,204,51},
      thickness=0.5));
  connect(val1.port_1, buiHea.ports_bHeaWat[1]) annotation (Line(points={{166,
          -142},{292,-142},{292,-5.06667},{278,-5.06667}}, color={0,127,255}));
  connect(buiHea.PPum, acLoad.Pow) annotation (Line(points={{281.533,19.2},{294,
          19.2},{294,20},{308,20},{308,-404},{302,-404},{302,-418}}, color={0,0,
          127}));
  connect(TSetHea.y, conHeaPID1.u_s) annotation (Line(points={{394.6,44},{397.6,
          44},{397.6,43},{400.6,43}}, color={0,0,127}));
  connect(gaiHea.u, conHeaPID1.y) annotation (Line(points={{426.8,44},{421.75,
          44},{421.75,43},{416.7,43}}, color={0,0,127}));
  connect(sinZonFlo.TRooAir, conHeaPID1.u_m) annotation (Line(points={{241,4.2},
          {372,4.2},{372,60},{409,60},{409,34.6}}, color={0,0,127}));
  connect(conCooPID.y, gaiCoo.u) annotation (Line(points={{418.8,16},{418.8,18},
          {426.8,18}}, color={0,0,127}));
  connect(conCooPID.u_m, sinZonFlo.TRooAir)
    annotation (Line(points={{410,6.4},{410,4.2},{241,4.2}}, color={0,0,127}));
  connect(TSetCoo.y, conCooPID.u_s) annotation (Line(points={{394.6,18},{394.6,
          16},{400.4,16}}, color={0,0,127}));
  connect(booleanExpression.y, NeoTowerControl.UseChp) annotation (Line(points={{-674.9,
          -319},{-644,-319},{-644,-316.28},{-624.88,-316.28}},        color={
          255,0,255}));
  connect(ChpControl.TopTlayer, tan.T3) annotation (Line(points={{-651.76,55.44},
          {-676,55.44},{-676,-70},{-500,-70},{-500,-72},{17.54,-72},{17.54,
          11.16}},                                     color={0,0,127}));
  connect(NeoTowerControl.TopTlayer, tan.T3) annotation (Line(points={{-624.88,
          -293.28},{-640,-293.28},{-640,-100},{-520,-100},{-520,-96},{17.54,-96},
          {17.54,11.16}},
                   color={0,0,127}));
  connect(NeoTowerControl.LowTlayer, tan.T8) annotation (Line(points={{-624.88,
          -343.88},{-640,-343.88},{-640,-296},{-644,-296},{-644,-32},{-240,-32},
          {-240,-96},{-216,-96},{-216,-104},{40,-104},{40,-30.73},{18.13,-30.73}},
        color={0,0,127}));
  connect(ChpControl.LowTlayer, tan.T8) annotation (Line(points={{-651.76,9.24},
          {-651.76,-32},{-240,-32},{-240,-96},{-216,-96},{-216,-104},{40,-104},
          {40,-30.73},{18.13,-30.73}}, color={0,0,127}));
  connect(ChpControl.CHPON, wolfCGB50_GC1.CBOn1) annotation (Line(points={{
          -561.88,42},{-480,42},{-480,41.76},{-464.72,41.76}}, color={255,0,255}));
  connect(realExpression.y, wolfCGB50_GC1.ControlIn) annotation (Line(points={{
          -483,6},{-476,6},{-476,24},{-480,24},{-480,31.26},{-461.78,31.26}},
        color={0,0,127}));
  connect(val4.port_1, wolfCGB50_GC1.port_b) annotation (Line(points={{-162,
          -140},{-244,-140},{-244,35.04},{-362.24,35.04}}, color={0,127,255}));
  connect(val3.port_1, wolfCGB50_GC1.port_a) annotation (Line(points={{-230,-20},
          {-324,-20},{-324,9},{-360.56,9}}, color={0,127,255}));
  connect(wolfCGB50_GC1.weaBus, weaDat.weaBus) annotation (Line(
      points={{-536.96,60.24},{-472,60.24},{-472,148},{-40,148},{-40,130},{-48,
          130}},
      color={255,204,51},
      thickness=0.5));
  connect(wolfCGB50_GC1.term_p, gri.terminal) annotation (Line(points={{-342.08,
          -6.96},{-342.08,-244},{-284,-244},{-284,-530},{50,-530}}, color={0,
          120,120}));
  connect(neoTower5_GC1.CHPOn, NeoTowerControl.CHPON) annotation (Line(points={
          {-390.12,-279.48},{-480,-279.48},{-480,-308},{-526.44,-308}}, color={
          255,0,255}));
  connect(neoTower5_GC1.weaBus, weaDat.weaBus) annotation (Line(
      points={{-497.76,-268.56},{-497.76,-272},{-412,-272},{-412,130},{-48,130}},
      color={255,204,51},
      thickness=0.5));

  connect(realExpression1.y, neoTower5_GC1.CHPModulation) annotation (Line(
        points={{-415,-326},{-404,-326},{-404,-311.72},{-390.12,-311.72}},
        color={0,0,127}));
  connect(val4.port_3, neoTower5_GC1.port_b) annotation (Line(points={{-152,
          -150},{-152,-164},{-264.8,-164},{-264.8,-275.84}}, color={0,127,255}));
  connect(val3.port_3, neoTower5_GC1.port_a) annotation (Line(points={{-220,-30},
          {-220,-160},{-280,-160},{-280,-308.08},{-262.72,-308.08}}, color={0,
          127,255}));
  connect(neoTower5_GC1.term_p, gri.terminal) annotation (Line(points={{-244,
          -325.76},{-244,-530},{50,-530}}, color={0,120,120}));
  connect(val3.port_2, tan.fluPorVol[2]) annotation (Line(points={{-210,-20},{
          -132,-20},{-132,-11.26},{-74.5,-11.26}}, color={0,127,255}));
  connect(val4.port_2, tan.fluPorVol[9]) annotation (Line(points={{-142,-140},{
          -132,-140},{-132,5.26},{-74.5,5.26}}, color={0,127,255}));
  connect(val1.port_2, tan.fluPorVol1[8]) annotation (Line(points={{146,-142},{
          28,-142},{28,-108},{44,-108},{44,12},{5.74,12},{5.74,4.08}}, color={0,
          127,255}));
  connect(acLoad1.Pow,domHotWatTan. PEle) annotation (Line(points={{168,-422},{
          168,-392},{184,-392},{184,-344},{175.6,-344}},
                                   color={0,0,127}));
  connect(conTSetHot.y,domHotWatTan. TDomSet) annotation (Line(points={{68.7,
          -345},{82.55,-345},{82.55,-344},{96.4,-344}},
                                                     color={0,0,127}));
  connect(domHotWatTan.port_aDom,souCol1. ports[1]) annotation (Line(points={{100,
          -322.4},{68,-322.4},{68,-287},{60,-287}},
        color={0,127,255}));
  connect(souCol1.ports[2], theMixVal.port_col) annotation (Line(points={{60,-285},
          {60,-288},{236,-288},{236,-238.4},{254,-238.4}},
                    color={0,127,255}));
  connect(theMixVal.port_hot,domHotWatTan. port_bDom) annotation (Line(points={{254,
          -231.2},{184,-231.2},{184,-322.4},{172,-322.4}},     color={0,127,255}));
  connect(val1.port_3, domHotWatTan.port_aHea) annotation (Line(points={{156,
          -132},{156,-128},{176,-128},{176,-340},{188,-340},{188,-365.6},{172,
          -365.6}}, color={0,127,255}));
  connect(buiHea.ports_aHeaWat[1], val2.port_1) annotation (Line(points={{172,
          -5.06667},{116,-5.06667},{116,0},{60,0},{60,-24},{70,-24}}, color={0,
          127,255}));
  connect(val2.port_3, domHotWatTan.port_bHea) annotation (Line(points={{80,-34},
          {80,-268},{32,-268},{32,-365.6},{100,-365.6}}, color={0,127,255}));
  connect(val2.port_2, tan.fluPorVol1[2]) annotation (Line(points={{90,-24},{
          100,-24},{100,4},{44,4},{44,12},{5.74,12},{5.74,-10.08}}, color={0,
          127,255}));
  connect(sinZonFlo.ports[1], buiHea.ports_aHeaWat[2]) annotation (Line(points=
          {{212.3,-18},{194,-18},{194,-1.33333},{172,-1.33333}}, color={0,127,
          255}));
  connect(sinZonFlo.ports[2], buiHea.ports_bHeaWat[2]) annotation (Line(points=
          {{213.3,-18},{246,-18},{246,-1.33333},{278,-1.33333}}, color={0,127,
          255}));
  connect(val2.y, y1.y) annotation (Line(points={{80,-12},{80,12},{120,12},{120,
          28},{113,28}}, color={0,0,127}));
  connect(booleanExpression1.y, ChpControl.UseChp) annotation (Line(points={{
          -690.9,33},{-668,33},{-668,34.44},{-651.76,34.44}}, color={255,0,255}));
                                                      annotation (Line(points={{-754.64,38.96},{-786.32,38.96},{-786.32,40},{-818,
          40}}, color={255,0,255}),
    Icon(
      coordinateSystem(
        preserveAspectRatio=false, extent={{-460,-500},{380,120}})),
    Diagram(
        coordinateSystem(
        preserveAspectRatio=false, extent={{-460,-500},{380,120}})),
    __Dymola_Commands(
      file="modelica://Buildings/Resources/Scripts/Dymola/DHC/Loads/Heating/Examples/BuildingTimeSeriesWithETS.mos" "Simulate and plot"),
    experiment(
      StopTime=19500,
      Interval=1,
      Tolerance=1e-06,
      __Dymola_Algorithm="Dassl"),
    Documentation(info="<html>
<p>
This model provides an example for a building with loads provided
as time series and connected to a direct ETS for heating with the
return heating water temperature controlled below a maximum threshold.
</p>
</html>", revisions="<html>
<ul>
<li>
March 20, 2022, by Chengnan Shi:<br/>
First implementation.
</li>
</ul>
</html>"));
end MF5_indep;
