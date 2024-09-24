within ProsNet.Prosumers;
model SF1 "Example model of a building with loads provided as time series and
  connected to an ETS for cooling"
  extends Modelica.Icons.Example;
  package Medium=ProsNet.Fluid.Building_Fluid.Utili.Media.Water
    "Medium model";
  Modelica.Blocks.Sources.RealExpression realExpression(y=0.5)
    annotation (Placement(transformation(extent={{-558,54},{-538,74}})));
  Modelica.Blocks.Sources.RealExpression realExpression1(y=0.5)
    annotation (Placement(transformation(extent={{-546,-188},{-526,-168}})));
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
        origin={449,-175})));

  Modelica.Blocks.Sources.Constant conTSetMix(k(
      final unit="K",
      displayUnit="degC") = 308.15)
    "Temperature setpoint for mixed water supply to fixture"
    annotation (Placement(transformation(extent={{-8,-8},{8,8}},
        rotation=180,
        origin={448,-148})));

  Modelica.Blocks.Sources.Ramp y1(
    height=1,
    duration=1,
    offset=0) "Control signal"
    annotation (Placement(transformation(extent={{92,18},{112,38}})));
  Modelica.Blocks.Sources.Constant conTSetHot(k(
      final unit="K",
      displayUnit="degC") = 313.15)
    "Temperature setpoint for hot water supply to fixture"
    annotation (Placement(transformation(extent={{8,-8},{-8,8}},
        rotation=0,
        origin={48,-300})));
  Fluid.Pumps.FlowControlled_m_flow pump(m_flow_nominal=10, addPowerToMedium=
        true)
    annotation (Placement(transformation(extent={{-118,-298},{-74,-264}})));
  Components.Generators.Digital_Twins.WolfCGB14_GC wolfCGB14_GC1
    annotation (Placement(transformation(extent={{-524,2},{-432,56}})));
  Components.Generators.Digital_Twins.NeoTower2_GC neoTower2_GC1
    annotation (Placement(transformation(extent={{-528,-194},{-436,-130}})));
  Fluid.Sources.Boundary_pT souCol(redeclare package Medium =
        ProsNet.Media.Water, nPorts=2)
    annotation (Placement(transformation(extent={{170,-196},{190,-176}})));

  Components.Generators.Laboratory_Models.PVSimpleOriented pv(
    A=200e3/800/0.12,
    til=0.34906585039887,
    azi=-0.78539816339745,
    V_nominal=480)
    annotation (Placement(transformation(extent={{476,-370},{564,-278}})));
  Components.Generators.Laboratory_Models.ASHRAE93 solCol(
    redeclare package Medium = ProsNet.Fluid.Building_Fluid.Utili.Media.Water,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
    massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    nSeg=9,
    azi=0.010646508437165,
    til=0.054803338512622,
    rho=0.2,
    nColType=ProsNet.Fluid.Building_Fluid.Utili.Fluid.SolarCollectors.Types.NumberSelection.Area,
    totalArea=1.312,
    sysConfig=ProsNet.Fluid.Building_Fluid.Utili.Fluid.SolarCollectors.Types.SystemConfiguration.Series,
    per=ProsNet.Fluid.Building_Fluid.Utili.Fluid.SolarCollectors.Data.GlazedFlatPlate.FP_GuangdongFSPTY95())
    annotation (Placement(transformation(extent={{-424,-362},{-332,-272}})));

  Components.Consumers.ThermostaticMixingValve theMixVal(mMix_flow_nominal=1.2*
        datWatHea.mDom_flow_nominal)
    annotation (Placement(transformation(extent={{254,-242},{294,-206}})));
  Components.Consumers.StorageTankWithExternalHeatExchanger domHotWatTan(
    redeclare package MediumDom =
        ProsNet.Fluid.Building_Fluid.Utili.Media.Water,
    redeclare package MediumHea =
        ProsNet.Fluid.Building_Fluid.Utili.Media.Water,
    dat=datWatHea)
    annotation (Placement(transformation(extent={{76,-336},{146,-264}})));
  Components.Electrical.Inductive acLoad2(linearized=false, mode=ProsNet.Fluid.Building_Fluid.Utili.Electrical.Types.Load.VariableZ_P_input)
    annotation (Placement(transformation(
        extent={{-22,-17},{22,17}},
        rotation=90,
        origin={-53,-446})));
  Components.Electrical.Inductive acLoad1(linearized=false, mode=ProsNet.Fluid.Building_Fluid.Utili.Electrical.Types.Load.VariableZ_P_input)
    annotation (Placement(transformation(
        extent={{-24,-16},{24,16}},
        rotation=90,
        origin={168,-446})));
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
    annotation (Placement(transformation(extent={{440,-228},{460,-208}})));
  Controls.BooleanToReal booToRea(realTrue=2.5, realFalse=0)
    annotation (Placement(transformation(extent={{-132,-244},{-112,-224}})));
  Components.Valves.ThreeWayLinear val3(
  redeclare package Medium = ProsNet.Fluid.Building_Fluid.Utili.Media.Water,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
    use_inputFilter=false,
    m_flow_nominal=2,
    dpValve_nominal=1,
    l={0.05,0.05})
    annotation (Placement(transformation(extent={{-310,-28},{-290,-8}})));
  Components.Valves.ThreeWayLinear val4(
    redeclare package Medium = Fluid.Building_Fluid.Utili.Media.Water,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
    use_inputFilter=false,
    m_flow_nominal=2,
    dpValve_nominal=1,
    l={0.05,0.05})
    annotation (Placement(transformation(extent={{-310,-134},{-290,-114}})));
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
        ModelicaServices.ExternalReferences.loadResource(
        "modelica://ProsNet/Data/weatherdata/DEU_Munich.108660_IWEC.mos"))
    annotation (Placement(transformation(extent={{-68,120},{-48,140}})));
  Components.Storage.Stratified tan(
    redeclare package Medium = ProsNet.Fluid.Building_Fluid.Utili.Media.Water,
    m_flow_nominal=2,
    show_T=true,
    VTan(displayUnit="l") = 0.75,
    hTan=2,
    dIns=0.3,
    kIns=0.2,
    nSeg=10,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
    p_start=100000,
    T_start=351.15,
    TFlu_start={353.15,348.15,345.15,340.15,336.15,330.15,326.15,321.15,317.15,
        314.15})
    annotation (Placement(transformation(extent={{-104,-62},{14,56}})));
  Modelica.Blocks.Sources.BooleanExpression booleanExpression(y=true)
    annotation (Placement(transformation(extent={{-758,-58},{-736,-36}})));
  Controls.GeneratorControlPackage.ControlGenerator.GeneratorControl NeoTowerControl(TStartCHP
      =338, TStopCHP=340)
    annotation (Placement(transformation(extent={{-682,-214},{-614,-146}})));
  Controls.GeneratorControlPackage.ControlGenerator.GeneratorControl ChpControl(TStartCHP
      =333, TStopCHP=336)
    annotation (Placement(transformation(extent={{-694,6},{-628,72}})));
  Components.Consumers.BuildingRCZ1Valve buildingRCZ1Valve(
    facMul=18,
    nZon=3,
    nPorts_aChiWat=1,
    nPorts_bChiWat=1,
    nPorts_aHeaWat=1,
    nPorts_bHeaWat=1)
    annotation (Placement(transformation(extent={{242,-102},{326,-16}})));
  Modelica.Blocks.Interfaces.RealOutput TTop
    "Absolute temperature as output signal"
    annotation (Placement(transformation(extent={{328,202},{366,240}}),
        iconTransformation(extent={{328,202},{366,240}})));
  Modelica.Blocks.Interfaces.RealOutput TMiddle
    "Absolute temperature as output signal"
    annotation (Placement(transformation(extent={{330,164},{366,200}}),
        iconTransformation(extent={{330,164},{366,200}})));
  Modelica.Blocks.Interfaces.RealOutput TBottom
    "Absolute temperature as output signal"
    annotation (Placement(transformation(extent={{330,134},{364,168}}),
        iconTransformation(extent={{330,134},{364,168}})));
  Modelica.Blocks.Sources.Ramp y2(
    height=1,
    duration=1,
    offset=0) "Control signal"
    annotation (Placement(transformation(extent={{-192,244},{-172,264}})));
  Modelica.Fluid.Interfaces.FluidPort_a port_a2
    "Fluid connector a (positive design flow direction is from port_a to port_b)"
    annotation (Placement(transformation(extent={{-368,246},{-182,404}}),
        iconTransformation(extent={{-330,194},{-144,352}})));
  Modelica.Fluid.Interfaces.FluidPort_b port_b2
    "Fluid connector b (positive design flow direction is from port_a to port_b)"
    annotation (Placement(transformation(extent={{-174,304},{24,472}}),
        iconTransformation(extent={{-136,252},{62,420}})));
  Controls.GeneratorControlPackage.ControlGenerator.HeatFlowRate
    heatFlowRate_CB
    annotation (Placement(transformation(extent={{-414,144},{-368,190}})));
  Modelica.Fluid.Sensors.MassFlowRate massFlowRate_Sup_gen(redeclare package
      Medium = Modelica.Media.Water.ConstantPropertyLiquidWater) annotation (
      Placement(transformation(
        extent={{-11,11},{11,-11}},
        rotation=0,
        origin={-355,1})));
  Modelica.Fluid.Sensors.TemperatureTwoPort T_sens_sec_hot(redeclare package
      Medium = Modelica.Media.Water.ConstantPropertyLiquidWater,
      allowFlowReversal=true)                       annotation (Placement(
        transformation(
        extent={{-14,-9},{14,9}},
        rotation=0,
        origin={-414,55})));
  Modelica.Fluid.Sensors.TemperatureTwoPort T_ret_gen(redeclare package Medium =
        Modelica.Media.Water.ConstantPropertyLiquidWater, allowFlowReversal=
        true) annotation (Placement(transformation(
        extent={{-14,-9},{14,9}},
        rotation=180,
        origin={-394,1})));
  Modelica.Fluid.Sensors.MassFlowRate massFlowRate_Sup_gen1(redeclare package
      Medium = Modelica.Media.Water.ConstantPropertyLiquidWater) annotation (
      Placement(transformation(
        extent={{-11,11},{11,-11}},
        rotation=0,
        origin={153,39})));
  Modelica.Fluid.Sensors.TemperatureTwoPort T_sens_sec_hot1(redeclare package
      Medium = Modelica.Media.Water.ConstantPropertyLiquidWater,
      allowFlowReversal=true)                       annotation (Placement(
        transformation(
        extent={{-14,-9},{14,9}},
        rotation=0,
        origin={198,41})));
  Modelica.Fluid.Sensors.TemperatureTwoPort T_ret_gen1(redeclare package Medium =
        Modelica.Media.Water.ConstantPropertyLiquidWater, allowFlowReversal=
        true) annotation (Placement(transformation(
        extent={{-14,-9},{14,9}},
        rotation=180,
        origin={362,1})));
  Controls.GeneratorControlPackage.ControlGenerator.HeatFlowRate
    heatFlowRate_consume
    annotation (Placement(transformation(extent={{434,34},{480,80}})));
  Controls.GeneratorControlPackage.ControlGenerator.HeatFlowRate
    heatFlowRate_Chp
    annotation (Placement(transformation(extent={{-642,-330},{-596,-284}})));
  Modelica.Fluid.Sensors.TemperatureTwoPort T_sens_sec_hot2(redeclare package
      Medium = Modelica.Media.Water.ConstantPropertyLiquidWater,
      allowFlowReversal=true)                       annotation (Placement(
        transformation(
        extent={{-14,-9},{14,9}},
        rotation=0,
        origin={-414,-119})));
  Modelica.Fluid.Sensors.MassFlowRate massFlowRate_Sup_gen2(redeclare package
      Medium = Modelica.Media.Water.ConstantPropertyLiquidWater) annotation (
      Placement(transformation(
        extent={{-11,11},{11,-11}},
        rotation=0,
        origin={-307,-197})));
  Modelica.Fluid.Sensors.TemperatureTwoPort T_ret_gen2(redeclare package Medium =
        Modelica.Media.Water.ConstantPropertyLiquidWater, allowFlowReversal=
        true) annotation (Placement(transformation(
        extent={{-14,-9},{14,9}},
        rotation=180,
        origin={-380,-201})));
  Components.Valves.ThreeWayLinear A(
    redeclare package Medium = Fluid.Building_Fluid.Utili.Media.Water,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
    use_inputFilter=false,
    m_flow_nominal=2,
    dpValve_nominal(displayUnit="kPa") = 5000,
    l={0.05,0.05})
    annotation (Placement(transformation(extent={{86,142},{106,162}})));
  Components.Valves.ThreeWayLinear B(
    redeclare package Medium = Fluid.Building_Fluid.Utili.Media.Water,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
    use_inputFilter=false,
    m_flow_nominal=2,
    dpValve_nominal(displayUnit="kPa") = 5000,
    l={0.05,0.05})
    annotation (Placement(transformation(extent={{82,366},{102,386}})));
equation
  connect(wolfCGB14_GC1.ControlIn, realExpression.y) annotation (Line(points={{
          -516.737,42.2545},{-537,42.2545},{-537,64}},color={0,0,127}));
  connect(neoTower2_GC1.CHPModulation, realExpression1.y) annotation (Line(
        points={{-512.188,-143.673},{-512.188,-160},{-525,-160},{-525,-178}},
                  color={0,0,127}));

  connect(pump.port_a, solCol.port_b) annotation (Line(points={{-118,-281},{-312,
          -281},{-312,-317},{-332,-317}},                         color={0,127,255}));
  connect(souCol.ports[1], theMixVal.port_col) annotation (Line(points={{190,
          -187},{200,-187},{200,-238.4},{254,-238.4}},
                                                 color={0,127,255}));
  connect(theMixVal.TMixSet, conTSetMix.y) annotation (Line(points={{252,-220.4},
          {244,-220.4},{244,-196},{468,-196},{468,-132},{432,-132},{432,-148},{439.2,
          -148}}, color={0,0,127}));
  connect(sch.y[1], theMixVal.yMixSet) annotation (Line(points={{439.1,-175},{240,
          -175},{240,-209.6},{252,-209.6}}, color={0,0,127}));
  connect(conTSetHot.y, domHotWatTan.TDomSet)
    annotation (Line(points={{39.2,-300},{72.5,-300}}, color={0,0,127}));
  connect(theMixVal.port_hot, domHotWatTan.port_bDom) annotation (Line(points={{
          254,-231.2},{160,-231.2},{160,-278.4},{146,-278.4}}, color={0,127,255}));
  connect(solCol.port_a, domHotWatTan.port_bHea) annotation (Line(points={{-424,
          -317},{-438,-317},{-438,-386},{64,-386},{64,-321.6},{76,-321.6}},
                        color={0,127,255}));
  connect(domHotWatTan.port_aHea, pump.port_b) annotation (Line(points={{146,-321.6},
          {152,-321.6},{152,-260},{-60,-260},{-60,-281},{-74,-281}}, color={0,127,
          255}));
  connect(acLoad2.Pow, pump.P) annotation (Line(points={{-53,-424},{-53,-265.7},
          {-71.8,-265.7}}, color={0,0,127}));
  connect(acLoad1.Pow, domHotWatTan.PEle) annotation (Line(points={{168,-422},{168,
          -300},{149.5,-300}}, color={0,0,127}));
  connect(pv.terminal, conv.terminal_p) annotation (Line(points={{476,-324},{464,
          -324},{464,-608},{16,-608}}, color={0,0,255}));
  connect(bat.terminal, conv.terminal_p) annotation (Line(points={{-470,-610},{-488,
          -610},{-488,-664},{28,-664},{28,-608},{16,-608}}, color={0,0,255}));
  connect(con.SOC, bat.SOC) annotation (Line(points={{-232,-628},{-348,-628},{-348,
          -584.8},{-369.2,-584.8}}, color={0,0,127}));
  connect(con.y, bat.P) annotation (Line(points={{-197,-628},{-188,-628},{-188,-540},
          {-422,-540},{-422,-568}}, color={0,0,127}));
  connect(wolfCGB14_GC1.term_p, gri.terminal) annotation (Line(points={{-422.8,9.85455},
          {-436,9.85455},{-436,10},{-448,10},{-448,-528},{-446,-528},{-446,-530},
          {50,-530}}, color={0,120,120}));
  connect(acLoad1.terminal, gri.terminal) annotation (Line(points={{168,-470},{168,
          -532},{110,-532},{110,-530},{50,-530}}, color={0,120,120}));
  connect(conv.terminal_n, gri.terminal) annotation (Line(points={{-72,-608},{-84,
          -608},{-84,-530},{50,-530}}, color={0,120,120}));
  connect(acLoad2.terminal, gri.terminal) annotation (Line(points={{-53,-468},{-53,
          -530},{50,-530}}, color={0,120,120}));
  connect(neoTower2_GC1.term_p, gri.terminal) annotation (Line(points={{-439.45,
          -192.255},{-448,-192.255},{-448,-434},{-446,-434},{-446,-530},{50,
          -530}},
        color={0,120,120}));
  connect(booToRea.u, domHotWatTan.charge) annotation (Line(points={{-134,-234},
          {-142,-234},{-142,-348},{153,-348},{153,-332.4}}, color={255,0,255}));
  connect(booToRea.y, pump.m_flow_in) annotation (Line(points={{-110,-234},{-98,
          -234},{-98,-260.6},{-96,-260.6}}, color={0,0,127}));
  connect(val3.y, y.y)
    annotation (Line(points={{-300,-6},{-304,-6},{-304,40},{-256,40},{-256,66},
          {-269,66}},                                        color={0,0,127}));
  connect(val4.y, y.y) annotation (Line(points={{-300,-112},{-300,-48},{-269,
          -48},{-269,66}},
        color={0,0,127}));
  connect(val1.port_3, domHotWatTan.port_aDom) annotation (Line(points={{156,
          -132},{156,-128},{36,-128},{36,-278},{76,-278},{76,-278.4}},   color=
          {0,127,255}));
  connect(val1.y, y1.y) annotation (Line(points={{156,-154},{156,-164},{120,-164},
          {120,28},{113,28}},                    color={0,0,127}));
  connect(pv.weaBus, weaDat.weaBus) annotation (Line(
      points={{520,-282.6},{520,130},{-48,130}},
      color={255,204,51},
      thickness=0.5));
  connect(weaDat.weaBus, solCol.weaBus) annotation (Line(
      points={{-48,130},{-248,130},{-248,-248},{-424,-248},{-424,-273.8}},
      color={255,204,51},
      thickness=0.5));
  connect(wolfCGB14_GC1.weaBus, weaDat.weaBus) annotation (Line(
      points={{-559.832,39.3091},{-559.832,24},{-600,24},{-600,130},{-48,130}},
      color={255,204,51},
      thickness=0.5));

  connect(weaDat.weaBus, neoTower2_GC1.weaBus) annotation (Line(
      points={{-48,130},{-600,130},{-600,-120.691},{-570.55,-120.691}},
      color={255,204,51},
      thickness=0.5));
  connect(val1.port_2, tan.fluPorVol1[10]) annotation (Line(points={{146,-142},
          {88,-142},{88,-16},{2,-16},{2,10},{5.74,10},{5.74,8.8}},
                                                              color={0,127,255}));
  connect(booleanExpression.y, ChpControl.UseChp) annotation (Line(points={{-734.9,
          -47},{-724,-47},{-724,33.06},{-703.24,33.06}}, color={255,0,255}));
  connect(booleanExpression.y, NeoTowerControl.UseChp) annotation (Line(points=
          {{-734.9,-47},{-724,-47},{-724,-186.12},{-691.52,-186.12}}, color={
          255,0,255}));
  connect(ChpControl.TopTlayer, tan.T3) annotation (Line(points={{-703.24,49.56},
          {-720,49.56},{-720,96},{-608,96},{-608,-16},{-592,-16},{-592,-104},{-520,
          -104},{-520,-96},{17.54,-96},{17.54,11.16}}, color={0,0,127}));
  connect(NeoTowerControl.TopTlayer, tan.T3) annotation (Line(points={{-691.52,
          -169.12},{-708,-169.12},{-708,0},{-700,0},{-700,-4},{-608,-4},{-608,
          -16},{-592,-16},{-592,-104},{-520,-104},{-520,-96},{17.54,-96},{17.54,
          11.16}}, color={0,0,127}));
  connect(NeoTowerControl.LowTlayer, tan.T8) annotation (Line(points={{-691.52,
          -206.52},{-708,-206.52},{-708,-168},{-704,-168},{-704,-32},{-240,-32},
          {-240,-96},{-216,-96},{-216,-104},{40,-104},{40,-30.73},{18.13,-30.73}},
        color={0,0,127}));
  connect(ChpControl.LowTlayer, tan.T8) annotation (Line(points={{-703.24,13.26},
          {-703.24,-32},{-240,-32},{-240,-96},{-216,-96},{-216,-104},{40,-104},
          {40,-30.73},{18.13,-30.73}}, color={0,0,127}));
  connect(NeoTowerControl.CHPON, neoTower2_GC1.CHPOn) annotation (Line(points={{-618.76,
          -180},{-564,-180},{-564,-124.473},{-512.188,-124.473}},
        color={255,0,255}));
  connect(ChpControl.CHPON, wolfCGB14_GC1.CBOn1) annotation (Line(points={{-632.62,
          39},{-592,39},{-592,88},{-519.158,88},{-519.158,29.9818}},
        color={255,0,255}));
  connect(buildingRCZ1Valve.ports_aChiWat[1], souCol.ports[2]) annotation (Line(
        points={{242,-84.8},{232,-84.8},{232,-185},{190,-185}}, color={0,127,
          255}));
  connect(buildingRCZ1Valve.ports_bChiWat[1], val1.port_1) annotation (Line(
        points={{326,-84.8},{344,-84.8},{344,-142},{166,-142}}, color={0,127,
          255}));
  connect(buildingRCZ1Valve.weaBus, weaDat.weaBus) annotation (Line(
      points={{284.14,-28.3267},{284.14,130},{-48,130}},
      color={255,204,51},
      thickness=0.5));
  connect(tan.T1, TTop) annotation (Line(points={{18.72,27.68},{34,27.68},{34,
          222},{354,222},{354,221},{347,221}},
                                  color={0,0,127}));
  connect(tan.T5, TMiddle) annotation (Line(points={{18.13,-4.77},{24,-4.77},{
          24,182},{348,182}},              color={0,0,127}));
  connect(tan.T10, TBottom) annotation (Line(points={{18.13,-47.25},{216,-47.25},
          {216,150},{348,150},{348,151},{347,151}},
                                            color={0,0,127}));
  connect(TTop, TTop)
    annotation (Line(points={{347,221},{347,221}}, color={0,0,127}));
  connect(port_b2,port_b2)
    annotation (Line(points={{-75,388},{-75,388}},
                                                 color={0,127,255}));
  connect(val3.port_2, tan.fluPorVol[1]) annotation (Line(points={{-290,-18},{
          -240,-18},{-240,-24},{-200,-24},{-200,-32},{-128,-32},{-128,-8},{-104,
          -8},{-104,-13.62},{-74.5,-13.62}}, color={0,127,255}));
  connect(val4.port_2, tan.fluPorVol[10]) annotation (Line(points={{-290,-124},
          {-290,-128},{-224,-128},{-224,-24},{-128,-24},{-128,-8},{-104,-8},{
          -104,7.62},{-74.5,7.62}}, color={0,127,255}));
  connect(T_ret_gen.port_b, wolfCGB14_GC1.port_a) annotation (Line(points={{-408,1},
          {-440.716,1},{-440.716,22.6182}},         color={0,127,255}));
  connect(T_sens_sec_hot.port_a, wolfCGB14_GC1.port_b) annotation (Line(points={{-428,55},
          {-440,55},{-440,42.7455},{-447.011,42.7455}},            color={0,127,
          255}));
  connect(T_ret_gen.T, heatFlowRate_CB.Thot) annotation (Line(points={{-394,
          -8.9},{-394,-16},{-368,-16},{-368,24},{-376,24},{-376,136},{-448,136},
          {-448,177.12},{-421.82,177.12}}, color={0,0,127}));
  connect(T_sens_sec_hot.T, heatFlowRate_CB.Tcold) annotation (Line(points={{
          -414,64.9},{-414,72},{-432,72},{-432,136},{-440,136},{-440,164.7},{
          -422.28,164.7}}, color={0,0,127}));
  connect(massFlowRate_Sup_gen.m_flow, heatFlowRate_CB.mdot) annotation (Line(
        points={{-355,-11.1},{-355,-16},{-376,-16},{-376,40},{-384,40},{-384,
          120},{-422.28,120},{-422.28,153.66}},
                            color={0,0,127}));
  connect(massFlowRate_Sup_gen1.port_b, T_sens_sec_hot1.port_a) annotation (
      Line(points={{164,39},{174,39},{174,41},{184,41}}, color={0,127,255}));
  connect(T_sens_sec_hot1.port_b, buildingRCZ1Valve.ports_aHeaWat[1])
    annotation (Line(points={{212,41},{232,41},{232,0},{224,0},{224,-67.6},{242,
          -67.6}}, color={0,127,255}));
  connect(buildingRCZ1Valve.ports_bHeaWat[1], T_ret_gen1.port_a) annotation (
      Line(points={{326,-67.6},{400,-67.6},{400,1},{376,1}}, color={0,127,255}));
  connect(massFlowRate_Sup_gen1.m_flow, heatFlowRate_consume.mdot) annotation (
      Line(points={{153,26.9},{153,16},{336,16},{336,43.66},{425.72,43.66}},
        color={0,0,127}));
  connect(T_sens_sec_hot1.T, heatFlowRate_consume.Thot) annotation (Line(points
        ={{198,50.9},{198,67.12},{426.18,67.12}}, color={0,0,127}));
  connect(heatFlowRate_consume.Tcold, T_ret_gen1.T) annotation (Line(points={{
          425.72,54.7},{416,54.7},{416,-16},{392,-16},{392,-8.9},{362,-8.9}},
        color={0,0,127}));
  connect(T_sens_sec_hot2.port_a, neoTower2_GC1.port_b) annotation (Line(points={{-428,
          -119},{-440,-119},{-440,-150.945},{-454.4,-150.945}},       color={0,
          127,255}));
  connect(T_ret_gen2.port_b, neoTower2_GC1.port_a) annotation (Line(points={{-394,
          -201},{-416,-201},{-416,-168.982},{-453.25,-168.982}},      color={0,
          127,255}));
  connect(heatFlowRate_Chp.mdot, massFlowRate_Sup_gen2.m_flow) annotation (Line(
        points={{-650.28,-320.34},{-656,-320.34},{-656,-232},{-600,-232},{-600,
          -216},{-376,-216},{-376,-209.1},{-307,-209.1}}, color={0,0,127}));
  connect(val4.port_1, T_sens_sec_hot.port_b) annotation (Line(points={{-310,
          -124},{-310,-128},{-328,-128},{-328,0},{-336,0},{-336,80},{-400,80},{
          -400,55}}, color={0,127,255}));
  connect(val3.port_1, massFlowRate_Sup_gen.port_b) annotation (Line(points={{
          -310,-18},{-344,-18},{-344,1}}, color={0,127,255}));
  connect(T_ret_gen.port_a, massFlowRate_Sup_gen.port_a) annotation (Line(
        points={{-380,1},{-373,1},{-373,1},{-366,1}}, color={0,127,255}));
  connect(val4.port_3, T_sens_sec_hot2.port_b) annotation (Line(points={{-300,
          -134},{-304,-134},{-304,-152},{-400,-152},{-400,-119}}, color={0,127,
          255}));
  connect(massFlowRate_Sup_gen2.port_b, val3.port_3) annotation (Line(points={{
          -296,-197},{-264,-197},{-264,-28},{-300,-28}}, color={0,127,255}));
  connect(massFlowRate_Sup_gen2.port_a, T_ret_gen2.port_a) annotation (Line(
        points={{-318,-197},{-342,-197},{-342,-201},{-366,-201}}, color={0,127,
          255}));
  connect(heatFlowRate_Chp.Thot, T_ret_gen2.T) annotation (Line(points={{
          -649.82,-296.88},{-664,-296.88},{-664,-224},{-608,-224},{-608,-210.9},
          {-380,-210.9}}, color={0,0,127}));
  connect(T_sens_sec_hot2.T, heatFlowRate_Chp.Tcold) annotation (Line(points={{
          -414,-109.1},{-414,-104},{-480,-104},{-480,-120},{-536,-120},{-536,
          -136},{-560,-136},{-560,-344},{-664,-344},{-664,-309.3},{-650.28,
          -309.3}}, color={0,0,127}));
  connect(port_a2, A.port_3) annotation (Line(points={{-275,325},{48,325},{48,
          152},{72,152},{72,142},{96,142}}, color={0,127,255}));
  connect(A.port_1, tan.fluPorVol1[1]) annotation (Line(points={{86,152},{86,
          104},{48,104},{48,-12.44},{5.74,-12.44}}, color={0,127,255}));
  connect(massFlowRate_Sup_gen1.port_a, A.port_2) annotation (Line(points={{142,
          39},{128,39},{128,152},{106,152}}, color={0,127,255}));
  connect(y2.y, A.y) annotation (Line(points={{-171,254},{-171,240},{-144,240},
          {-144,256},{64,256},{64,192},{96,192},{96,164}}, color={0,0,127}));
  connect(B.port_3, port_b2) annotation (Line(points={{92,366},{92,320},{40,320},
          {40,288},{-75,288},{-75,388}}, color={0,127,255}));
  connect(B.port_1, tan.fluPorVol1[10]) annotation (Line(points={{82,376},{48,
          376},{48,320},{40,320},{40,8.8},{5.74,8.8}}, color={0,127,255}));
  connect(T_ret_gen1.port_b, B.port_2) annotation (Line(points={{348,1},{344,1},
          {344,120},{288,120},{288,376},{102,376}}, color={0,127,255}));
  connect(B.y, y2.y) annotation (Line(points={{92,388},{92,400},{64,400},{64,
          254},{-171,254}}, color={0,0,127}));
                                                      annotation (Line(points={{-754.64,38.96},{-786.32,38.96},{-786.32,40},{-818,
          40}}, color={255,0,255}),
    Icon(
      coordinateSystem(
        preserveAspectRatio=false, extent={{-460,-500},{380,120}}), graphics={
        Rectangle(
          extent={{-444,-64},{392,-474}},
          lineColor={0,0,0},
          lineThickness=1),
        Polygon(
          points={{392,-64},{64,118},{-114,116},{-442,-62},{392,-64}},
          lineColor={0,0,0},
          lineThickness=1),
        Text(
          extent={{-328,-72},{288,-458}},
          textColor={28,108,200},
          textString="SF1")}),
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
end SF1;
