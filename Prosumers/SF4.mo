within ProsNet.Prosumers;
model SF4 "Example model of a building with loads provided as time series and
  connected to an ETS for cooling"
  extends Modelica.Icons.Example;
  package Medium=Fluid.Building_Fluid.Utili.Media.Water
    "Medium model";

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

  Components.Electrical.BatteryControl con
    annotation (Placement(transformation(extent={{-220,-574},{-188,-542}})));
  Components.Electrical.Battery bat(EMax=500e3*4*3600, V_nominal=480)
    annotation (Placement(transformation(extent={{-48,-42},{48,42}},
        rotation=180,
        origin={-66,-496})));
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
    VTan(displayUnit="l") = 0.75,
    hTan=3,
    dIns=0.3,
    nSeg=10,
    T_start=353.15,
    TFlu_start={353.15,348.15,345.15,340.15,336.15,330.15,326.15,321.15,317.15,
        314.15},
    tau=1)
    annotation (Placement(transformation(extent={{-72,-72},{50,50}})));
  Fluid.Sources.Boundary_pT souCol1(redeclare package Medium = Media.Water,
      nPorts=2)
    annotation (Placement(transformation(extent={{76,-258},{96,-238}})));
  Components.Consumers.GenericDomesticHotWaterWithHeatExchanger datWatHea(VTan=0.1892706,
      mDom_flow_nominal=6.52944E-06*1000)
    annotation (Placement(transformation(extent={{218,-168},{238,-148}})));
  Components.Consumers.ThermostaticMixingValve theMixVal(mMix_flow_nominal=1.2*
        datWatHea.mDom_flow_nominal)
    annotation (Placement(transformation(extent={{280,-276},{320,-240}})));

  Components.BoundryCondition.ReaderTMY3 weaDat1(filNam=
        ModelicaServices.ExternalReferences.loadResource("modelica://ProsNet/Data/weatherdata/USA_CA_San.Francisco.Intl.AP.724940_TMY3.mos"))
    annotation (Placement(transformation(extent={{168,102},{188,122}})));
  Components.Generators.Digital_Twins.WolfCGB20_GC wolfCGB20_GC1
    annotation (Placement(transformation(extent={{-438,-20},{-350,38}})));
  Modelica.Blocks.Sources.RealExpression realExpression(y=1)
    annotation (Placement(transformation(extent={{-526,-4},{-506,16}})));
  Modelica.Blocks.Sources.Ramp y(
    height=1,
    duration=1,
    offset=0) "Control signal"
    annotation (Placement(transformation(extent={{-320,82},{-300,102}})));
  Fluid.Valves.TwoWayEqualPercentage val(
    redeclare package Medium = Media.Water,
    m_flow_nominal=10,
    dpValve_nominal=6000)
    annotation (Placement(transformation(extent={{-278,16},{-258,36}})));
  Modelica.Blocks.Sources.Constant conTSetHot1(k(
      final unit="K",
      displayUnit="degC") = 353.15)
    "Temperature setpoint for hot water supply to fixture"
    annotation (Placement(transformation(extent={{-394,-292},{-376,-274}})));
  Components.Generators.ElectricHeater eleHea(
    redeclare package Medium = Fluid.Building_Fluid.Utili.Media.Water,
    m_flow_nominal=1000,
    dp_nominal=6000,
    eta=0.95,
    QMax_flow=1000,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial)
    annotation (Placement(transformation(extent={{-314,-344},{-254,-286}})));
  Components.Electrical.Inductive acLoad2(linearized=false, mode=ProsNet.Fluid.Building_Fluid.Utili.Electrical.Types.Load.VariableZ_P_input)
    annotation (Placement(transformation(
        extent={{-24,-16},{24,16}},
        rotation=90,
        origin={-110,-404})));
  Controls.GeneratorControlPackage.ControlGenerator.GeneratorControl
    generatorControl(TStartCHP=329, TStopCHP=333)
    annotation (Placement(transformation(extent={{-644,-36},{-570,38}})));
  Modelica.Blocks.Sources.BooleanExpression booleanExpression(y=true)
    annotation (Placement(transformation(extent={{-738,-18},{-716,4}})));
  Controls.GeneratorControlPackage.ControlGenerator.ElHeaterControl
    elHeaterControl(deltaTEHon=5, Tmax_TS=363)
    annotation (Placement(transformation(extent={{-560,-364},{-496,-300}})));
  Modelica.Blocks.Sources.BooleanExpression booleanExpression1(y=true)
    annotation (Placement(transformation(extent={{-618,-350},{-596,-328}})));
  Components.Consumers.BuildingRCZ1Valve buildingRCZ1Valve(
    facMul=18,
    nZon=3,
    nPorts_aChiWat=1,
    nPorts_bHeaWat=1,
    nPorts_bChiWat=1,
    nPorts_aHeaWat=1)
    annotation (Placement(transformation(extent={{216,-8},{300,78}})));
  Components.Valves.ThreeWayLinear val1(
    redeclare package Medium = Fluid.Building_Fluid.Utili.Media.Water,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
    use_inputFilter=false,
    m_flow_nominal=2,
    dpValve_nominal=1,
    l={0.05,0.05})
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=180,
        origin={254,-34})));
  Modelica.Blocks.Sources.Ramp y1(
    height=1,
    duration=1,
    offset=0) "Control signal"
    annotation (Placement(transformation(extent={{302,-116},{322,-96}})));
  Modelica.Blocks.Interfaces.RealOutput TTop
    "Absolute temperature as output signal"
    annotation (Placement(transformation(extent={{538,224},{576,262}}),
        iconTransformation(extent={{328,202},{366,240}})));
  Modelica.Blocks.Interfaces.RealOutput TMiddle
    "Absolute temperature as output signal"
    annotation (Placement(transformation(extent={{540,186},{576,222}}),
        iconTransformation(extent={{330,164},{366,200}})));
  Modelica.Blocks.Interfaces.RealOutput TBottom
    "Absolute temperature as output signal"
    annotation (Placement(transformation(extent={{540,156},{574,190}}),
        iconTransformation(extent={{330,134},{364,168}})));
  Modelica.Blocks.Sources.Ramp y2(
    height=1,
    duration=1,
    offset=0) "Control signal"
    annotation (Placement(transformation(extent={{-182,304},{-162,324}})));
  Modelica.Fluid.Interfaces.FluidPort_a port_a2
    "Fluid connector a (positive design flow direction is from port_a to port_b)"
    annotation (Placement(transformation(extent={{-356,266},{-170,424}}),
        iconTransformation(extent={{-330,194},{-144,352}})));
  Modelica.Fluid.Interfaces.FluidPort_b port_b2
    "Fluid connector b (positive design flow direction is from port_a to port_b)"
    annotation (Placement(transformation(extent={{54,264},{252,432}}),
        iconTransformation(extent={{-136,252},{62,420}})));
  Fluid.Valves.TwoWayEqualPercentage         val5(
    redeclare package Medium = Media.Water,
    m_flow_nominal=10,
    dpValve_nominal=6000)
    annotation (Placement(transformation(extent={{-54,152},{-34,172}})));
equation

  connect(con.SOC, bat.SOC) annotation (Line(points={{-222,-558},{-232,-558},{
          -232,-521.2},{-118.8,-521.2}},             color={0,0,127}));
  connect(con.y, bat.P) annotation (Line(points={{-187,-558},{-187,-560},{-66,
          -560},{-66,-538}},                         color={0,0,127}));
  connect(conv.terminal_p, bat.terminal) annotation (Line(points={{158,-504},{
          172,-504},{172,-552},{-4,-552},{-4,-496},{-18,-496}},
                                                         color={0,0,255}));
  connect(gri.terminal, conv.terminal_n) annotation (Line(points={{18,-430},{18,
          -444},{60,-444},{60,-504},{70,-504}}, color={0,120,120}));
  connect(pv.terminal, conv.terminal_p) annotation (Line(points={{436,-328},{
          432,-328},{432,-504},{158,-504}}, color={0,0,255}));
  connect(acLoad1.terminal, gri.terminal) annotation (Line(points={{244,-386},{
          244,-444},{18,-444},{18,-430}},                     color={0,120,120}));
  connect(acLoad1.Pow, domHotWatTan.PEle) annotation (Line(points={{244,-338},{244,
          -306},{211.6,-306}},     color={0,0,127}));
  connect(conTSetHot.y, domHotWatTan.TDomSet) annotation (Line(points={{104.7,-307},
          {104.7,-306},{132.4,-306}},                color={0,0,127}));
  connect(domHotWatTan.port_aHea, tan.fluPorVol1[8]) annotation (Line(points={{
          208,-327.6},{212,-327.6},{212,-356},{41.46,-356},{41.46,-3.68}},
        color={0,127,255}));
  connect(domHotWatTan.port_bHea, tan.fluPorVol1[2]) annotation (Line(points={{136,
          -327.6},{98,-327.6},{98,-328},{58,-328},{58,0},{41.46,0},{41.46,
          -18.32}},                                   color={0,127,255}));
  connect(souCol1.ports[1], theMixVal.port_col) annotation (Line(points={{96,-249},
          {166,-249},{166,-248},{192,-248},{192,-260},{268,-260},{268,-272.4},{
          280,-272.4}},
                    color={0,127,255}));
  connect(theMixVal.port_hot, domHotWatTan.port_bDom) annotation (Line(points={{
          280,-265.2},{276,-265.2},{276,-284.4},{208,-284.4}}, color={0,127,255}));
  connect(theMixVal.TMixSet, conTSetMix.y) annotation (Line(points={{278,-254.4},
          {196,-254.4},{196,-79},{214.9,-79}}, color={0,0,127}));
  connect(sch.y[1], theMixVal.yMixSet) annotation (Line(points={{212.8,-114},{212.8,
          -116},{204,-116},{204,-243.6},{278,-243.6}}, color={0,0,127}));
  connect(weaDat1.weaBus, pv.weaBus) annotation (Line(
      points={{188,112},{469,112},{469,-295.6}},
      color={255,204,51},
      thickness=0.5));

  connect(weaDat1.weaBus, wolfCGB20_GC1.weaBus) annotation (Line(
      points={{188,112},{-494,112},{-494,70},{-493.147,70},{-493.147,29.88}},
      color={255,204,51},
      thickness=0.5));
  connect(wolfCGB20_GC1.term_p, gri.terminal) annotation (Line(points={{
          -355.867,-12.46},{-355.867,-10},{-360,-10},{-360,-446},{18,-446},{18,
          -430}},
        color={0,120,120}));
  connect(realExpression.y, wolfCGB20_GC1.ControlIn) annotation (Line(points={{-505,6},
          {-473.113,6},{-473.113,9.87},{-441.227,9.87}},    color={0,0,127}));
  connect(wolfCGB20_GC1.port_b, val.port_a) annotation (Line(points={{-369.947,
          16.54},{-369.947,16},{-370,16},{-370,26},{-278,26}},
                                             color={0,127,255}));
  connect(val.y, y.y)
    annotation (Line(points={{-268,38},{-268,92},{-299,92}}, color={0,0,127}));
  connect(eleHea.TSet,conTSetHot1. y) annotation (Line(points={{-320,-291.8},{
          -324,-291.8},{-324,-290},{-352,-290},{-352,-283},{-375.1,-283}},
        color={0,0,127}));
  connect(acLoad2.Pow, eleHea.P) annotation (Line(points={{-110,-380},{-112,
          -380},{-112,-332.4},{-251,-332.4}}, color={0,0,127}));
  connect(acLoad2.terminal, gri.terminal) annotation (Line(points={{-110,-428},
          {-112,-428},{-112,-446},{18,-446},{18,-430}}, color={0,120,120}));
  connect(booleanExpression.y, generatorControl.UseChp) annotation (Line(points={{-714.9,
          -7},{-676,-7},{-676,-5.66},{-654.36,-5.66}},         color={255,0,255}));
  connect(generatorControl.CHPON, wolfCGB20_GC1.CBOn1) annotation (Line(points={{-575.18,
          1},{-540,1},{-540,24},{-500,24},{-500,20},{-456,20},{-456,17.12},{
          -438.587,17.12}},         color={255,0,255}));
  connect(val.port_b, tan.fluPorVol[10]) annotation (Line(points={{-258,26},{
          -92,26},{-92,8},{-41.5,8},{-41.5,-0.02}}, color={0,127,255}));
  connect(tan.T3, generatorControl.TopTlayer) annotation (Line(points={{53.66,
          3.64},{72,3.64},{72,72},{-676,72},{-676,12.84},{-654.36,12.84}},
        color={0,0,127}));
  connect(wolfCGB20_GC1.port_a, tan.fluPorVol[2]) annotation (Line(points={{
          -368.773,-1.44},{-100,-1.44},{-100,8},{-41.5,8},{-41.5,-19.54}},
        color={0,127,255}));
  connect(elHeaterControl.ON, eleHea.on) annotation (Line(points={{-500.48,-332},
          {-344,-332},{-344,-306.3},{-320,-306.3}}, color={255,0,255}));
  connect(booleanExpression1.y, elHeaterControl.Use) annotation (Line(points={{
          -594.9,-339},{-594.9,-337.76},{-568.96,-337.76}}, color={255,0,255}));
  connect(elHeaterControl.Tlayer5, tan.T5) annotation (Line(points={{-568.96,
          -321.76},{-584,-321.76},{-584,-264},{64,-264},{64,-72},{68,-72},{68,
          -36},{72,-36},{72,-12.83},{54.27,-12.83}}, color={0,0,127}));
  connect(weaDat1.weaBus, buildingRCZ1Valve.weaBus) annotation (Line(
      points={{188,112},{258.14,112},{258.14,65.6733}},
      color={255,204,51},
      thickness=0.5));
  connect(buildingRCZ1Valve.ports_bHeaWat[1], tan.fluPorVol1[8]) annotation (
      Line(points={{300,26.4},{304,26.4},{304,-12},{204,-12},{204,-20},{180,-20},
          {180,-56},{84,-56},{84,-88},{41.46,-88},{41.46,-3.68}}, color={0,127,
          255}));
  connect(buildingRCZ1Valve.ports_aChiWat[1], souCol1.ports[2]) annotation (
      Line(points={{216,9.2},{212,9.2},{212,-32},{184,-32},{184,-232},{116,-232},
          {116,-247},{96,-247}}, color={0,127,255}));
  connect(buildingRCZ1Valve.ports_bChiWat[1], val1.port_1) annotation (Line(
        points={{300,9.2},{312,9.2},{312,-34},{264,-34}}, color={0,127,255}));
  connect(domHotWatTan.port_aDom, val1.port_2) annotation (Line(points={{136,
          -284.4},{124,-284.4},{124,-236},{192,-236},{192,-34},{244,-34}},
        color={0,127,255}));
  connect(y1.y, val1.y) annotation (Line(points={{323,-106},{323,-108},{332,
          -108},{332,-64},{254,-64},{254,-46}}, color={0,0,127}));
  connect(val1.port_3, tan.fluPorVol1[8]) annotation (Line(points={{254,-24},{
          254,-20},{180,-20},{180,-56},{92,-56},{92,-64},{84,-64},{84,-88},{
          41.46,-88},{41.46,-3.68}}, color={0,127,255}));
  connect(tan.T1,TTop)  annotation (Line(points={{54.88,20.72},{244,20.72},{244,
          244},{564,244},{564,243},{557,243}},
                                  color={0,0,127}));
  connect(tan.T5,TMiddle)  annotation (Line(points={{54.27,-12.83},{528,-12.83},
          {528,204},{558,204}},            color={0,0,127}));
  connect(tan.T10,TBottom)  annotation (Line(points={{54.27,-56.75},{80,-56.75},
          {80,-16},{212,-16},{212,84},{524,84},{524,173},{557,173}},
                                            color={0,0,127}));
  connect(TTop, TTop)
    annotation (Line(points={{557,243},{557,243}}, color={0,0,127}));
  connect(port_b2,port_b2)
    annotation (Line(points={{153,348},{153,348}},
                                                 color={0,127,255}));
  connect(port_a2, tan.fluPorVol1[2]) annotation (Line(points={{-263,345},{-263,
          120},{104,120},{104,24},{184,24},{184,-56},{88,-56},{88,-88},{41.46,
          -88},{41.46,-18.32}}, color={0,127,255}));
  connect(y2.y, val5.y) annotation (Line(points={{-161,314},{-44,314},{-44,174}},
        color={0,0,127}));
  connect(val5.port_b, port_b2) annotation (Line(points={{-34,162},{153,162},{
          153,348}}, color={0,127,255}));
  connect(val5.port_a, tan.fluPorVol1[4]) annotation (Line(points={{-54,162},{
          -72,162},{-72,120},{104,120},{104,24},{184,24},{184,-56},{88,-56},{88,
          -88},{41.46,-88},{41.46,-13.44}},color={0,127,255}));
  connect(buildingRCZ1Valve.ports_aHeaWat[1], tan.fluPorVol1[1]) annotation (
      Line(points={{216,26.4},{184,26.4},{184,-56},{88,-56},{88,-88},{41.46,-88},
          {41.46,-20.76}},color={0,127,255}));
  connect(eleHea.port_a, tan.fluPorVol[10]) annotation (Line(points={{-314,-315},
          {-352,-315},{-352,-40},{-336,-40},{-336,0},{-104,0},{-104,8},{-41.5,8},
          {-41.5,-0.02}}, color={0,127,255}));
  connect(eleHea.port_b, tan.fluPorVol[1]) annotation (Line(points={{-254,-315},
          {-232,-315},{-232,-272},{-352,-272},{-352,-40},{-336,-40},{-336,0},{
          -88,0},{-88,8},{-41.5,8},{-41.5,-21.98}}, color={0,127,255}));
  connect(generatorControl.LowTlayer, tan.T6) annotation (Line(points={{-654.36,
          -27.86},{-664,-27.86},{-664,56},{-448,56},{-448,64},{54.27,64},{54.27,
          -21.37}}, color={0,0,127}));
  annotation (
    Icon(
      coordinateSystem(
        preserveAspectRatio=false, extent={{-460,-480},{380,120}}), graphics={
        Rectangle(
          extent={{-456,-60},{380,-470}},
          lineColor={0,0,0},
          lineThickness=1),
        Polygon(
          points={{380,-62},{52,120},{-126,118},{-454,-60},{380,-62}},
          lineColor={0,0,0},
          lineThickness=1),
        Text(
          extent={{-396,-68},{372,-418}},
          textColor={28,108,200},
          textString="SF4")}),
    Diagram(
        coordinateSystem(
        preserveAspectRatio=false, extent={{-460,-480},{380,120}})),
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
end SF4;
