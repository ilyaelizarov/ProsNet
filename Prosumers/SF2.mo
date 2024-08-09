within ProsNet.Prosumers;
model SF2 "Example model of a building with loads provided as time series and
  connected to an ETS for cooling"
  extends Modelica.Icons.Example;
  package Medium=ProsNet.Fluid.Building_Fluid.Utili.Media.Water
    "Medium model";
  Modelica.Blocks.Sources.RealExpression realExpression(y=1)
    annotation (Placement(transformation(extent={{-504,-4},{-484,16}})));
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
    annotation (Placement(transformation(extent={{88,54},{108,74}})));

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
    T_start=353.15,
    TFlu_start={353.15,349.15,345.15,342.15,338.15,334.15,330.15,326.15,322.15,
        318.15})
    annotation (Placement(transformation(extent={{-104,-62},{14,56}})));
  Components.Consumers.SingleZoneFloor sinZonFlo(redeclare package Medium =
        Modelica.Media.Water.ConstantPropertyLiquidWater, use_windPressure=
        false)
    annotation (Placement(transformation(extent={{204,-26},{244,14}})));
  Components.Consumers.BuildingTimeSeries buiHea(
    have_chiWat=false,
    filNam="modelica://ProsNet/SwissResidential_20190916.mos",
    nPorts_aHeaWat=3,
    nPorts_bHeaWat=3)
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
  Controls.GeneratorControlPackage.ControlGenerator.GeneratorControl ChpControl(TStartCHP
      =338, TStopCHP=343)
    annotation (Placement(transformation(extent={{-640,0},{-556,84}})));
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
  Components.Generators.Laboratory_Models.ASHRAE93 solCol(
    redeclare package Medium = Fluid.Building_Fluid.Utili.Media.Water,
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
    annotation (Placement(transformation(extent={{-560,-508},{-494,-446}})));

  Fluid.Pumps.FlowControlled_m_flow pump(m_flow_nominal=10, addPowerToMedium=
        true)
    annotation (Placement(transformation(extent={{-414,-480},{-370,-446}})));
  Controls.GeneratorControlPackage.ControlGenerator.SolarThermalControl
    solarThermalControl(deltaTonST=3, TCollectorST=323.15)
    annotation (Placement(transformation(extent={{-726,-504},{-648,-426}})));
  Controls.BooleanToReal booToRea(realTrue=2.5, realFalse=0)
    annotation (Placement(transformation(extent={{-428,-430},{-408,-410}})));
  Modelica.Blocks.Sources.BooleanExpression booleanExpression2(y=true)
    annotation (Placement(transformation(extent={{-786,-490},{-764,-468}})));
  Components.Generators.Digital_Twins.WolfCGB20_GC wolfCGB20_GC1
    annotation (Placement(transformation(extent={{-454,-8},{-350,62}})));
  Modelica.Blocks.Sources.RealExpression realExpression2(y=1)
    annotation (Placement(transformation(extent={{-584,-244},{-564,-224}})));
  Modelica.Blocks.Sources.RealExpression realExpression5(y=1)
    annotation (Placement(transformation(extent={{-554,-270},{-534,-250}})));
  Modelica.Blocks.Sources.BooleanExpression booleanExpression(y=true)
    annotation (Placement(transformation(extent={{-606,-224},{-586,-204}})));
  Components.Generators.Digital_Twins.WolfCHA10_GC wolfCHA10_GC1
    annotation (Placement(transformation(extent={{-540,-240},{-466,-166}})));
  Components.Valves.ThreeWayLinear valLin3(
    redeclare package Medium = Fluid.Building_Fluid.Utili.Media.Water,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
    use_inputFilter=false,
    m_flow_nominal=2,
    dpValve_nominal=6000,
    l={0.05,0.05})
    annotation (Placement(transformation(extent={{-322,-294},{-302,-274}})));
  Components.Valves.ThreeWayLinear valLin1(
    redeclare package Medium = Fluid.Building_Fluid.Utili.Media.Water,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
    use_inputFilter=false,
    m_flow_nominal=2,
    dpValve_nominal=6000,
    l={0.05,0.05})
    annotation (Placement(transformation(extent={{-196,-228},{-176,-208}})));
  Modelica.Blocks.Sources.Ramp y2(
    height=1,
    duration=1,
    offset=0) "Control signal"
    annotation (Placement(transformation(extent={{-340,-182},{-320,-162}})));
  Modelica.Fluid.Interfaces.FluidPort_a port_a1
    "Fluid connector a (positive design flow direction is from port_a to port_b)"
    annotation (Placement(transformation(extent={{30,204},{50,224}})));
  Modelica.Fluid.Interfaces.FluidPort_b port_b1
    "Fluid connector b (positive design flow direction is from port_a to port_b)"
    annotation (Placement(transformation(extent={{56,204},{76,224}})));
  Fluid.Valves.TwoWayEqualPercentage val(
    redeclare package Medium = Media.Water,
    m_flow_nominal=10,
    dpValve_nominal=6000)
    annotation (Placement(transformation(extent={{-196,-58},{-176,-38}})));
  Controls.GeneratorControlPackage.ControlGenerator.HPControl HP_ON(TStart=328,
      TStop=333)
    annotation (Placement(transformation(extent={{-708,-250},{-630,-172}})));
  Modelica.Blocks.Sources.BooleanExpression booleanExpression3(y=true)
    annotation (Placement(transformation(extent={{-782,-228},{-762,-208}})));
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
  connect(val1.y, y1.y) annotation (Line(points={{156,-154},{156,-156},{124,
          -156},{124,12},{122,12},{122,64},{109,64}},
                                                 color={0,0,127}));
  connect(pv.weaBus, weaDat.weaBus) annotation (Line(
      points={{520,-282.6},{520,130},{-48,130}},
      color={255,204,51},
      thickness=0.5));

  connect(sinZonFlo.weaBus, weaDat.weaBus) annotation (Line(
      points={{210.8,11},{132,11},{132,130},{-48,130}},
      color={255,204,51},
      thickness=0.5));
  connect(val1.port_1, buiHea.ports_bHeaWat[1]) annotation (Line(points={{166,
          -142},{292,-142},{292,-5.68889},{278,-5.68889}}, color={0,127,255}));
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
  connect(ChpControl.TopTlayer, tan.T3) annotation (Line(points={{-651.76,55.44},
          {-676,55.44},{-676,-70},{-500,-70},{-500,-72},{17.54,-72},{17.54,
          11.16}},                                     color={0,0,127}));
  connect(ChpControl.LowTlayer, tan.T8) annotation (Line(points={{-651.76,9.24},
          {-651.76,-32},{-240,-32},{-240,-104},{40,-104},{40,-30.73},{18.13,
          -30.73}},                    color={0,0,127}));

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
          -5.68889},{116,-5.68889},{116,-6},{58,-6},{58,-24},{70,-24}},
                                                                      color={0,
          127,255}));
  connect(val2.port_3, domHotWatTan.port_bHea) annotation (Line(points={{80,-34},
          {80,-268},{32,-268},{32,-365.6},{100,-365.6}}, color={0,127,255}));
  connect(val2.port_2, tan.fluPorVol1[2]) annotation (Line(points={{90,-24},{
          100,-24},{100,4},{44,4},{44,12},{5.74,12},{5.74,-10.08}}, color={0,
          127,255}));
  connect(sinZonFlo.ports[1], buiHea.ports_aHeaWat[2]) annotation (Line(points={{212.3,
          -18},{194,-18},{194,-3.2},{172,-3.2}},                 color={0,127,
          255}));
  connect(sinZonFlo.ports[2], buiHea.ports_bHeaWat[2]) annotation (Line(points={{213.3,
          -18},{246,-18},{246,-3.2},{278,-3.2}},                 color={0,127,
          255}));
  connect(val2.y, y1.y) annotation (Line(points={{80,-12},{80,48},{120,48},{120,
          64},{109,64}}, color={0,0,127}));
  connect(booleanExpression1.y, ChpControl.UseChp) annotation (Line(points={{
          -690.9,33},{-668,33},{-668,34.44},{-651.76,34.44}}, color={255,0,255}));
  connect(solCol.port_b,pump. port_a) annotation (Line(points={{-494,-477},{
          -428,-477},{-428,-463},{-414,-463}}, color={0,127,255}));
  connect(booToRea.y,pump. m_flow_in) annotation (Line(points={{-406,-420},{
          -406,-432},{-392,-432},{-392,-442.6}}, color={0,0,127}));
  connect(booToRea.u,solarThermalControl. ON) annotation (Line(points={{-430,
          -420},{-636,-420},{-636,-465},{-653.46,-465}}, color={255,0,255}));
  connect(solarThermalControl.Tlayer5, tan.T5) annotation (Line(points={{-736.92,
          -452.52},{-736.92,-418},{-736,-418},{-736,-384},{18.13,-384},{18.13,
          -4.77}},                                                color={0,0,
          127}));
  connect(booleanExpression2.y,solarThermalControl. Use) annotation (Line(
        points={{-762.9,-479},{-762.9,-472.02},{-736.92,-472.02}}, color={255,0,
          255}));
  connect(solCol.weaBus, weaDat.weaBus) annotation (Line(
      points={{-560,-447.24},{-560,-400},{-124,-400},{-124,130},{-48,130}},
      color={255,204,51},
      thickness=0.5));
  connect(wolfCGB20_GC1.weaBus, weaDat.weaBus) annotation (Line(
      points={{-519.173,52.2},{-516,52.2},{-516,130},{-48,130}},
      color={255,204,51},
      thickness=0.5));
  connect(gri.terminal, wolfCGB20_GC1.term_p) annotation (Line(points={{50,-530},
          {50,-536},{-248,-536},{-248,-352},{-288,-352},{-288,1.1},{-356.933,
          1.1}}, color={0,120,120}));
  connect(ChpControl.CHPON, wolfCGB20_GC1.CBOn1) annotation (Line(points={{-561.88,
          42},{-508.287,42},{-508.287,36.8},{-454.693,36.8}},         color={
          255,0,255}));
  connect(realExpression.y, wolfCGB20_GC1.ControlIn) annotation (Line(points={{-483,6},
          {-457.813,6},{-457.813,28.05}},         color={0,0,127}));
  connect(booleanExpression.y, wolfCHA10_GC1.HPMode) annotation (Line(points={{-585,
          -214},{-560,-214},{-560,-197.45},{-535.93,-197.45}},      color={255,
          0,255}));
  connect(realExpression2.y, wolfCHA10_GC1.HPModulation) annotation (Line(
        points={{-563,-234},{-552,-234},{-552,-212.25},{-535.93,-212.25}},
        color={0,0,127}));
  connect(realExpression5.y, wolfCHA10_GC1.HPAuxModulation) annotation (Line(
        points={{-533,-260},{-533,-243.155},{-535.93,-243.155},{-535.93,-226.31}},
        color={0,0,127}));
  connect(wolfCHA10_GC1.port_b, valLin3.port_1) annotation (Line(points={{
          -438.62,-191.16},{-438.62,-208},{-444,-208},{-444,-224},{-428,-224},{
          -428,-220},{-332,-220},{-332,-284},{-322,-284}}, color={0,127,255}));
  connect(wolfCHA10_GC1.port_a, valLin1.port_1) annotation (Line(points={{
          -437.14,-214.1},{-308,-214.1},{-308,-220},{-204,-220},{-204,-218},{
          -196,-218}}, color={0,127,255}));
  connect(y2.y, valLin3.y) annotation (Line(points={{-319,-172},{-312,-172},{
          -312,-272}}, color={0,0,127}));
  connect(y2.y, valLin1.y) annotation (Line(points={{-319,-172},{-312,-172},{
          -312,-206},{-186,-206}}, color={0,0,127}));
  connect(wolfCHA10_GC1.weaBus, weaDat.weaBus) annotation (Line(
      points={{-609.56,-176.36},{-609.56,-68},{-124,-68},{-124,130},{-48,130}},
      color={255,204,51},
      thickness=0.5));

  connect(wolfCHA10_GC1.term_p, gri.terminal) annotation (Line(points={{-420.86,
          -228.16},{-420.86,-380},{-248,-380},{-248,-536},{-84,-536},{-84,-530},
          {50,-530}}, color={0,120,120}));
  connect(valLin3.port_3, buiHea.ports_bHeaWat[3]) annotation (Line(points={{
          -312,-294},{-312,-304},{-280,-304},{-280,-196},{236,-196},{236,-144},
          {292,-144},{292,-0.711111},{278,-0.711111}}, color={0,127,255}));
  connect(valLin1.port_3, buiHea.ports_aHeaWat[3]) annotation (Line(points={{
          -186,-228},{-188,-228},{-188,-236},{112,-236},{112,0},{142,0},{142,
          -0.711111},{172,-0.711111}}, color={0,127,255}));
  connect(wolfCGB20_GC1.port_b, tan.fluPorVol[5]) annotation (Line(points={{
          -373.573,36.1},{-223.786,36.1},{-223.786,-4.18},{-74.5,-4.18}}, color
        ={0,127,255}));
  connect(pump.port_b, tan.fluPorVol[4]) annotation (Line(points={{-370,-463},{
          -252,-463},{-252,-400},{-132,-400},{-132,-6.54},{-74.5,-6.54}}, color
        ={0,127,255}));
  connect(solCol.port_a, tan.fluPorVol[5]) annotation (Line(points={{-560,-477},
          {-572,-477},{-572,-520},{-252,-520},{-252,-400},{-132,-400},{-132,
          -4.18},{-74.5,-4.18}}, color={0,127,255}));
  connect(valLin1.port_2, tan.fluPorVol[6]) annotation (Line(points={{-176,-218},
          {-176,-220},{-132,-220},{-132,-1.82},{-74.5,-1.82}}, color={0,127,255}));
  connect(valLin3.port_2, tan.fluPorVol[10]) annotation (Line(points={{-302,
          -284},{-132,-284},{-132,7.62},{-74.5,7.62}}, color={0,127,255}));
  connect(port_a1, tan.fluPorVol1[2]) annotation (Line(points={{40,214},{40,16},
          {5.74,16},{5.74,-10.08}}, color={0,127,255}));
  connect(port_b1, tan.fluPorVol1[4]) annotation (Line(points={{66,214},{66,-14},
          {8,-14},{8,-6},{5.74,-6},{5.74,-5.36}}, color={0,127,255}));
  connect(val.y, y.y) annotation (Line(points={{-186,-36},{-186,66},{-269,66}},
        color={0,0,127}));
  connect(val.port_a, wolfCGB20_GC1.port_a) annotation (Line(points={{-196,-48},
          {-296,-48},{-296,14.4},{-372.187,14.4}}, color={0,127,255}));
  connect(val.port_b, tan.fluPorVol[1]) annotation (Line(points={{-176,-48},{
          -136,-48},{-136,-13.62},{-74.5,-13.62}}, color={0,127,255}));
  connect(wolfCHA10_GC1.HPOn, HP_ON.ON) annotation (Line(points={{-535.93,
          -183.39},{-596,-183.39},{-596,-196},{-630,-196},{-630,-212},{-632,
          -212},{-632,-211},{-635.46,-211}}, color={255,0,255}));
  connect(tan.T2, HP_ON.TopTlayer) annotation (Line(points={{18.13,18.83},{24,
          18.83},{24,-128},{-728,-128},{-728,-152},{-736,-152},{-736,-198.52},{
          -718.92,-198.52}}, color={0,0,127}));
  connect(HP_ON.LowTlayer, tan.T6) annotation (Line(points={{-718.92,-241.42},{
          -736,-241.42},{-736,-200},{-728,-200},{-728,-152},{-600,-152},{-600,
          -112},{32,-112},{32,-13.03},{18.13,-13.03}}, color={0,0,127}));
  connect(booleanExpression3.y, HP_ON.Use) annotation (Line(points={{-761,-218},
          {-739.96,-218},{-739.96,-218.02},{-718.92,-218.02}}, color={255,0,255}));
                                                      annotation (Line(points={{-754.64,38.96},{-786.32,38.96},{-786.32,40},{-818,
          40}}, color={255,0,255}),
    Icon(
      coordinateSystem(
        preserveAspectRatio=false, extent={{-460,-500},{380,120}}), graphics={
          Bitmap(extent={{-414,-484},{404,240}}, fileName=
              "modelica://ProsNet/../thesis/report 7/New folder/sf2.JPG"),
        Rectangle(
          extent={{-416,-56},{420,-466}},
          lineColor={0,0,0},
          lineThickness=1),
        Polygon(
          points={{420,-56},{92,126},{-86,124},{-414,-54},{420,-56}},
          lineColor={0,0,0},
          lineThickness=1)}),
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
end SF2;