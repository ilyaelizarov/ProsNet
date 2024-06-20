within ProsNet.Prosumers;
model SF2_indep "Example model of a building with loads provided as time series and
  connected to an ETS for cooling"
  extends Modelica.Icons.Example;
  package Medium=Fluid.Building_Fluid.Utili.Media.Water
    "Medium model";
  Modelica.Blocks.Sources.RealExpression realExpression(y=1)
    annotation (Placement(transformation(extent={{-598,-46},{-578,-26}})));
  Modelica.Blocks.Sources.Ramp y(
    height=1,
    duration=1,
    offset=0) "Control signal"
    annotation (Placement(transformation(extent={{-240,54},{-220,74}})));

  Modelica.Blocks.Sources.Constant conTSetHot(k(
      final unit="K",
      displayUnit="degC") = 313.15)
    "Temperature setpoint for hot water supply to fixture"
    annotation (Placement(transformation(extent={{90,-314},{104,-300}})));
  Modelica.Blocks.Sources.CombiTimeTable sch(
    tableOnFile=true,
    tableName="tab1",
    fileName=Modelica.Utilities.Files.loadResource(
        "modelica://ProsNet/Data/Loads/HotWater/DHW_ApartmentMidRise.mos"),
    smoothness=Modelica.Blocks.Types.Smoothness.ContinuousDerivative,
    extrapolation=Modelica.Blocks.Types.Extrapolation.Periodic)
    "Domestic hot water fixture draw fraction schedule"
    annotation (Placement(transformation(extent={{-12,-12},{12,12}},
        rotation=180,
        origin={226,-114})));

  Modelica.Blocks.Sources.Constant conTSetMix(k(
      final unit="K",
      displayUnit="degC") = 308.15)
    "Temperature setpoint for mixed water supply to fixture"
    annotation (Placement(transformation(extent={{-11,-11},{11,11}},
        rotation=180,
        origin={227,-79})));

  Modelica.Blocks.Sources.RealExpression realExpression2(y=1)
    annotation (Placement(transformation(extent={{-488,-310},{-468,-290}})));
  Modelica.Blocks.Sources.RealExpression realExpression5(y=1)
    annotation (Placement(transformation(extent={{-490,-330},{-470,-310}})));
  Modelica.Blocks.Sources.BooleanExpression booleanExpression(y=true)
    annotation (Placement(transformation(extent={{-486,-280},{-466,-260}})));
  Modelica.Blocks.Sources.BooleanExpression booleanExpression1(y=true)
    annotation (Placement(transformation(extent={{-488,-260},{-468,-240}})));
  Modelica.Blocks.Sources.Ramp y2(
    height=1,
    duration=1,
    offset=0) "Control signal"
    annotation (Placement(transformation(extent={{-282,-256},{-250,-224}})));

  Components.Generators.Digital_Twins.WolfCGB20_GC wolfCGB20_GC1
    annotation (Placement(transformation(extent={{-534,-52},{-430,18}})));
  Components.Generators.Digital_Twins.WolfCHA10_GC wolfCHA10_GC1
    annotation (Placement(transformation(extent={{-444,-312},{-370,-238}})));
  Components.Electrical.BatteryControl con
    annotation (Placement(transformation(extent={{-74,-586},{-42,-554}})));
  Components.Electrical.Battery bat(EMax=500e3*4*3600, V_nominal=480)
    annotation (Placement(transformation(extent={{-308,-610},{-212,-526}})));
  Components.Electrical.ACDCConverter conv(conversionFactor=480/480, eta=0.9)
    annotation (Placement(transformation(extent={{70,-540},{158,-468}})));
  Components.Electrical.Grid gri(
    f=60,
    V=480,
    phiSou=0)
    annotation (Placement(transformation(extent={{-10,-430},{46,-382}})));
  Components.Generators.Laboratory_Models.PVSimpleOriented pv(
    A=200e3/800/0.12,
    til=0.34906585039887,
    azi=-0.78539816339745,
    V_nominal=480)
    annotation (Placement(transformation(extent={{436,-364},{502,-292}})));
  Components.Electrical.Inductive acLoad(linearized=false, mode=ProsNet.Fluid.Building_Fluid.Utili.Electrical.Types.Load.VariableZ_P_input)
    annotation (Placement(transformation(
        extent={{-24,-16},{24,16}},
        rotation=90,
        origin={336,-362})));
  Components.Electrical.Inductive acLoad1(linearized=false, mode=ProsNet.Fluid.Building_Fluid.Utili.Electrical.Types.Load.VariableZ_P_input)
    annotation (Placement(transformation(
        extent={{-24,-16},{24,16}},
        rotation=90,
        origin={244,-362})));
  Components.Consumers.DHW domHotWatTan(
    redeclare package MediumDom = Fluid.Building_Fluid.Utili.Media.Water,
    redeclare package MediumHea = Fluid.Building_Fluid.Utili.Media.Water,
    dat=datWatHea)
    annotation (Placement(transformation(extent={{136,-342},{208,-270}})));
  Components.Storage.Stratified tan(
    redeclare package Medium = Fluid.Building_Fluid.Utili.Media.Water,
    m_flow_nominal=2,
    VTan(displayUnit="l") = 0.775,
    hTan=3,
    dIns=0.3,
    nSeg=10,
    T_start=333.15)
    annotation (Placement(transformation(extent={{-72,-72},{50,50}})));
  Modelica.Blocks.Math.Gain gaiHea1(k=1E6)
                                          "Gain for heating"
    annotation (Placement(visible=true,transformation(origin={454,88},
                                                                     extent={{-6,-6},{6,6}},rotation=0)));
  Modelica.Blocks.Sources.Constant TSetHea1(k=273.15 + 20)
    "Set-point for heating"
    annotation (Placement(visible=true, transformation(origin={408,88},extent={{-6,-6},{6,6}},rotation=0)));
  Modelica.Blocks.Sources.Constant TSetCoo1(k=273.15 + 27)
    "Set-point for cooling"
    annotation (Placement(visible=true, transformation(origin={408,62},extent={{-6,-6},{6,6}},rotation=0)));
  Modelica.Blocks.Math.Gain gaiCoo1(k=-1E6)
                                           "Gain for cooling"
    annotation (Placement(visible=true,transformation(origin={454,62},
                                                                     extent={{-6,-6},{6,6}},rotation=0)));
  Components.Consumers.SingleZoneFloor sinZonFlo1(redeclare package Medium =
        Modelica.Media.Water.ConstantPropertyLiquidWater, use_windPressure=
        false)
    annotation (Placement(transformation(extent={{210,-8},{250,32}})));
  Components.Consumers.BuildingTimeSeries buiHea1(
    have_chiWat=false,
    filNam="modelica://ProsNet/SwissResidential_20190916.mos",
    nPorts_aHeaWat=3,
    nPorts_bHeaWat=3)
    annotation (Placement(transformation(extent={{190,-24},{296,88}})));
  Controls.LimPID conHeaPID1(
    k=0.1,
    Ti=300,
    reverseActing=true)
    annotation (Placement(transformation(extent={{422,80},{436,94}})));
  Controls.LimPID conCooPID1(
    k=0.1,
    Ti=300,
    reverseActing=true)
    annotation (Placement(transformation(extent={{422,52},{438,68}})));
  Components.Valves.ThreeWayLinear valLin2(
    redeclare package Medium = Fluid.Building_Fluid.Utili.Media.Water,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
    use_inputFilter=false,
    m_flow_nominal=2,
    dpValve_nominal=6000,
    l={0.05,0.05})
    annotation (Placement(transformation(extent={{-280,6},{-260,26}})));
  Components.Valves.ThreeWayLinear val3(
    redeclare package Medium = Fluid.Building_Fluid.Utili.Media.Water,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
    use_inputFilter=false,
    m_flow_nominal=2,
    dpValve_nominal=6000,
    l={0.05,0.05})
    annotation (Placement(transformation(extent={{-60,-126},{-40,-106}})));
  Components.Valves.ThreeWayLinear valLin4(
    redeclare package Medium = Fluid.Building_Fluid.Utili.Media.Water,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
    use_inputFilter=false,
    m_flow_nominal=2,
    dpValve_nominal=6000,
    l={0.05,0.05})
    annotation (Placement(transformation(extent={{-108,-368},{-88,-348}})));
  Components.Valves.ThreeWayLinear valLin3(
    redeclare package Medium = Fluid.Building_Fluid.Utili.Media.Water,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
    use_inputFilter=false,
    m_flow_nominal=2,
    dpValve_nominal=6000,
    l={0.05,0.05})
    annotation (Placement(transformation(extent={{-232,-334},{-212,-314}})));
  Fluid.Sources.Boundary_pT souCol1(redeclare package Medium = Media.Water,
      nPorts=2)
    annotation (Placement(transformation(extent={{94,-144},{114,-124}})));
  Components.Consumers.GenericDomesticHotWaterWithHeatExchanger datWatHea(VTan=0.1892706,
      mDom_flow_nominal=6.52944E-06*1000)
    annotation (Placement(transformation(extent={{218,-168},{238,-148}})));
  Components.Consumers.ThermostaticMixingValve theMixVal(mMix_flow_nominal=1.2*
        datWatHea.mDom_flow_nominal)
    annotation (Placement(transformation(extent={{280,-276},{320,-240}})));
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
    annotation (Placement(transformation(extent={{-418,-208},{-352,-146}})));

  Components.BoundryCondition.ReaderTMY3 weaDat1(filNam=
        ModelicaServices.ExternalReferences.loadResource("modelica://ProsNet/Data/weatherdata/USA_CA_San.Francisco.Intl.AP.724940_TMY3.mos"))
    annotation (Placement(transformation(extent={{172,140},{192,160}})));
  Modelica.Blocks.Sources.BooleanExpression booleanExpression2(y=false)
    annotation (Placement(transformation(extent={{-680,-26},{-632,30}})));
equation

  connect(wolfCGB20_GC1.ControlIn, realExpression.y) annotation (Line(points={{
          -537.813,-15.95},{-552,-15.95},{-552,-36},{-577,-36}},         color=
          {0,0,127}));
  connect(wolfCHA10_GC1.HPOn, booleanExpression1.y) annotation (Line(points={{
          -439.93,-255.39},{-439.93,-250},{-467,-250}}, color={255,0,255}));
  connect(wolfCHA10_GC1.HPMode, booleanExpression.y) annotation (Line(points={{
          -439.93,-269.45},{-452.465,-269.45},{-452.465,-270},{-465,-270}},
        color={255,0,255}));
  connect(wolfCHA10_GC1.HPModulation, realExpression2.y) annotation (Line(
        points={{-439.93,-284.25},{-460,-284.25},{-460,-300},{-467,-300}},
        color={0,0,127}));
  connect(wolfCHA10_GC1.HPAuxModulation, realExpression5.y) annotation (Line(
        points={{-439.93,-298.31},{-456,-298.31},{-456,-320},{-469,-320}},
        color={0,0,127}));
  connect(con.SOC, bat.SOC) annotation (Line(points={{-76,-570},{-76,-572},{
          -192,-572},{-192,-542.8},{-207.2,-542.8}}, color={0,0,127}));
  connect(con.y, bat.P) annotation (Line(points={{-41,-570},{-41,-572},{-32,
          -572},{-32,-508},{-260,-508},{-260,-526}}, color={0,0,127}));
  connect(conv.terminal_p, bat.terminal) annotation (Line(points={{158,-504},{
          172,-504},{172,-596},{-308,-596},{-308,-568}}, color={0,0,255}));
  connect(gri.terminal, wolfCHA10_GC1.term_p) annotation (Line(points={{18,-430},
          {18,-444},{-324.86,-444},{-324.86,-300.16}}, color={0,120,120}));
  connect(gri.terminal, wolfCGB20_GC1.term_p) annotation (Line(points={{18,-430},
          {18,-444},{-500,-444},{-500,-42.9},{-436.933,-42.9}}, color={0,120,
          120}));
  connect(gri.terminal, conv.terminal_n) annotation (Line(points={{18,-430},{18,
          -444},{60,-444},{60,-504},{70,-504}}, color={0,120,120}));
  connect(pv.terminal, conv.terminal_p) annotation (Line(points={{436,-328},{
          432,-328},{432,-504},{158,-504}}, color={0,0,255}));
  connect(acLoad.terminal, gri.terminal) annotation (Line(points={{336,-386},{
          336,-440},{60,-440},{60,-444},{18,-444},{18,-430}}, color={0,120,120}));
  connect(acLoad1.terminal, gri.terminal) annotation (Line(points={{244,-386},{244,
          -440},{60,-440},{60,-444},{18,-444},{18,-430}},     color={0,120,120}));
  connect(acLoad1.Pow, domHotWatTan.PEle) annotation (Line(points={{244,-338},{244,
          -306},{211.6,-306}},     color={0,0,127}));
  connect(conTSetHot.y, domHotWatTan.TDomSet) annotation (Line(points={{104.7,-307},
          {104.7,-306},{132.4,-306}},                color={0,0,127}));
  connect(domHotWatTan.port_aHea, tan.fluPorVol1[8]) annotation (Line(points={{
          208,-327.6},{212,-327.6},{212,-356},{41.46,-356},{41.46,-3.68}},
        color={0,127,255}));
  connect(domHotWatTan.port_bHea, tan.fluPorVol1[2]) annotation (Line(points={{
          136,-327.6},{41.46,-327.6},{41.46,-18.32}}, color={0,127,255}));
  connect(buiHea1.PPum, acLoad.Pow) annotation (Line(points={{299.533,43.2},{
          304,43.2},{304,8},{336,8},{336,-338}}, color={0,0,127}));
  connect(TSetHea1.y, conHeaPID1.u_s) annotation (Line(points={{414.6,88},{
          417.6,88},{417.6,87},{420.6,87}}, color={0,0,127}));
  connect(gaiHea1.u, conHeaPID1.y) annotation (Line(points={{446.8,88},{441.75,
          88},{441.75,87},{436.7,87}}, color={0,0,127}));
  connect(sinZonFlo1.TRooAir, conHeaPID1.u_m) annotation (Line(points={{247,
          22.2},{396,22.2},{396,104},{429,104},{429,78.6}}, color={0,0,127}));
  connect(conCooPID1.y, gaiCoo1.u) annotation (Line(points={{438.8,60},{438.8,
          62},{446.8,62}}, color={0,0,127}));
  connect(conCooPID1.u_m, sinZonFlo1.TRooAir) annotation (Line(points={{430,
          50.4},{430,22.2},{247,22.2}}, color={0,0,127}));
  connect(TSetCoo1.y, conCooPID1.u_s) annotation (Line(points={{414.6,62},{
          414.6,60},{420.4,60}}, color={0,0,127}));
  connect(buiHea1.ports_bHeaWat[1], tan.fluPorVol1[7]) annotation (Line(points={{296,
          18.3111},{304,18.3111},{304,0},{312,0},{312,-64},{88,-64},{88,-96},{
          41.46,-96},{41.46,-6.12}},       color={0,127,255}));
  connect(sinZonFlo1.ports[2], buiHea1.ports_bHeaWat[2]) annotation (Line(
        points={{219.3,-8.88178e-16},{219.3,14},{270,14},{270,0},{302,0},{302,
          20.8},{296,20.8}},       color={0,127,255}));
  connect(valLin2.y, y.y) annotation (Line(points={{-270,28},{-272,28},{-272,48},
          {-212,48},{-212,64},{-219,64}}, color={0,0,127}));
  connect(valLin2.port_1, wolfCGB20_GC1.port_a) annotation (Line(points={{-280,16},
          {-364,16},{-364,-29.6},{-452.187,-29.6}}, color={0,127,255}));
  connect(val3.y, y.y) annotation (Line(points={{-50,-104},{-52,-104},{-52,-92},
          {-96,-92},{-96,48},{-212,48},{-212,64},{-219,64}}, color={0,0,127}));
  connect(val3.port_3, wolfCGB20_GC1.port_b) annotation (Line(points={{-50,-126},
          {-52,-126},{-52,-136},{-172,-136},{-172,-88},{-392,-88},{-392,-7.9},{
          -453.573,-7.9}},
                  color={0,127,255}));
  connect(valLin4.y, y2.y) annotation (Line(points={{-98,-346},{-100,-346},{
          -100,-240},{-248.4,-240}},                    color={0,0,127}));
  connect(valLin4.port_1, wolfCHA10_GC1.port_a) annotation (Line(points={{-108,-358},
          {-108,-360},{-312,-360},{-312,-286.1},{-341.14,-286.1}}, color={0,127,
          255}));
  connect(valLin3.y, y2.y) annotation (Line(points={{-222,-312},{-224,-312},{
          -224,-302},{-236,-302},{-236,-240},{-248.4,-240}},
                                                        color={0,0,127}));
  connect(valLin3.port_1, wolfCHA10_GC1.port_b) annotation (Line(points={{-232,-324},
          {-308,-324},{-308,-263.16},{-342.62,-263.16}}, color={0,127,255}));
  connect(domHotWatTan.port_aDom, souCol1.ports[1]) annotation (Line(points={{136,
          -284.4},{92,-284.4},{92,-212},{140,-212},{140,-135},{114,-135}},
        color={0,127,255}));
  connect(souCol1.ports[2], theMixVal.port_col) annotation (Line(points={{114,-133},
          {140,-133},{140,-212},{92,-212},{92,-284},{132,-284},{132,-272.4},{280,
          -272.4}}, color={0,127,255}));
  connect(theMixVal.port_hot, domHotWatTan.port_bDom) annotation (Line(points={{
          280,-265.2},{276,-265.2},{276,-284.4},{208,-284.4}}, color={0,127,255}));
  connect(theMixVal.TMixSet, conTSetMix.y) annotation (Line(points={{278,-254.4},
          {196,-254.4},{196,-79},{214.9,-79}}, color={0,0,127}));
  connect(sch.y[1], theMixVal.yMixSet) annotation (Line(points={{212.8,-114},{212.8,
          -116},{204,-116},{204,-243.6},{278,-243.6}}, color={0,0,127}));
  connect(weaDat1.weaBus, sinZonFlo1.weaBus) annotation (Line(
      points={{192,150},{204,150},{204,100},{180,100},{180,29},{216.8,29}},
      color={255,204,51},
      thickness=0.5));
  connect(weaDat1.weaBus, pv.weaBus) annotation (Line(
      points={{192,150},{204,150},{204,108},{469,108},{469,-295.6}},
      color={255,204,51},
      thickness=0.5));
  connect(weaDat1.weaBus, wolfCHA10_GC1.weaBus) annotation (Line(
      points={{192,150},{204,150},{204,100},{-504,100},{-504,-248.36},{-513.56,-248.36}},
      color={255,204,51},
      thickness=0.5));

  connect(weaDat1.weaBus, wolfCGB20_GC1.weaBus) annotation (Line(
      points={{192,150},{204,150},{204,100},{-599.173,100},{-599.173,8.2}},
      color={255,204,51},
      thickness=0.5));
  connect(weaDat1.weaBus, solCol.weaBus) annotation (Line(
      points={{192,150},{204,150},{204,100},{-418,100},{-418,-147.24}},
      color={255,204,51},
      thickness=0.5));
  connect(wolfCHA10_GC1.port_a, buiHea1.ports_aHeaWat[1]) annotation (Line(
        points={{-341.14,-286.1},{-316,-286.1},{-316,-380},{-24,-380},{-24,-88},
          {88,-88},{88,-64},{168,-64},{168,18.3111},{190,18.3111}}, color={0,
          127,255}));
  connect(wolfCHA10_GC1.port_b, buiHea1.ports_bHeaWat[3]) annotation (Line(
        points={{-342.62,-263.16},{-296,-263.16},{-296,-172},{36,-172},{36,-96},
          {88,-96},{88,-64},{312,-64},{312,0},{304,0},{304,23.2889},{296,
          23.2889}}, color={0,127,255}));
  connect(valLin3.port_3, tan.fluPorVol[10]) annotation (Line(points={{-222,
          -334},{-224,-334},{-224,-372},{-116,-372},{-116,-374},{-74,-374},{-74,
          -176},{-148,-176},{-148,-12},{-92,-12},{-92,-0.02},{-41.5,-0.02}},
        color={0,127,255}));
  connect(val3.port_2, tan.fluPorVol[2]) annotation (Line(points={{-40,-116},{
          -32,-116},{-32,-84},{-84,-84},{-84,-20},{-41.5,-20},{-41.5,-19.54}},
        color={0,127,255}));
  connect(valLin2.port_2, tan.fluPorVol[1]) annotation (Line(points={{-260,16},
          {-100,16},{-100,-21.98},{-41.5,-21.98}}, color={0,127,255}));
  connect(solCol.port_a, tan.fluPorVol[3]) annotation (Line(points={{-418,-177},
          {-432,-177},{-432,-220},{-300,-220},{-300,-176},{-176,-176},{-176,-92},
          {-148,-92},{-148,-17.1},{-41.5,-17.1}}, color={0,127,255}));
  connect(solCol.port_b, tan.fluPorVol[4]) annotation (Line(points={{-352,-177},
          {-176,-177},{-176,-92},{-148,-92},{-148,-14.66},{-41.5,-14.66}},
        color={0,127,255}));
  connect(sinZonFlo1.ports[1], buiHea1.ports_aHeaWat[2]) annotation (Line(
        points={{218.3,-8.88178e-16},{218.3,16},{272,16},{272,0},{308,0},{308,
          -52},{168,-52},{168,20.8},{190,20.8}}, color={0,127,255}));
  connect(buiHea1.ports_aHeaWat[3], tan.fluPorVol1[1]) annotation (Line(points={{190,
          23.2889},{168,23.2889},{168,-64},{88,-64},{88,-88},{41.46,-88},{41.46,
          -20.76}},        color={0,127,255}));
  connect(valLin4.port_2, tan.fluPorVol[8]) annotation (Line(points={{-88,-358},
          {-88,-360},{-76,-360},{-76,-176},{-148,-176},{-148,-4.9},{-41.5,-4.9}},
        color={0,127,255}));
  connect(booleanExpression2.y, wolfCGB20_GC1.CBOn1) annotation (Line(points={{-629.6,
          2},{-608,2},{-608,-7.2},{-534.693,-7.2}},          color={255,0,255}));
  annotation (
    Icon(
      coordinateSystem(
        preserveAspectRatio=false, extent={{-460,-480},{380,120}})),
    Diagram(
        coordinateSystem(
        preserveAspectRatio=false, extent={{-460,-480},{380,120}})),
    __Dymola_Commands(
      file="modelica://Buildings/Resources/Scripts/Dymola/DHC/Loads/Heating/Examples/BuildingTimeSeriesWithETS.mos" "Simulate and plot"),
    experiment(
      StopTime=90000,
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
end SF2_indep;
