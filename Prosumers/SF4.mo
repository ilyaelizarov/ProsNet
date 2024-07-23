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
    T_start=353.15,
    TFlu_start={353.15,348.15,345.15,340.15,336.15,330.15,326.15,321.15,317.15,
        314.15})
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
    nPorts_bHeaWat=2,
    nPorts_aHeaWat=2)
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
    generatorControl(TStartCHP=329, TStopCHP=343)
    annotation (Placement(transformation(extent={{-644,-36},{-570,38}})));
  Modelica.Blocks.Sources.BooleanExpression booleanExpression(y=true)
    annotation (Placement(transformation(extent={{-738,-18},{-716,4}})));
  Controls.GeneratorControlPackage.ControlGenerator.ElHeaterControl
    elHeaterControl(deltaTEHon=5, Tmax_TS=363)
    annotation (Placement(transformation(extent={{-560,-364},{-496,-300}})));
  Modelica.Blocks.Sources.BooleanExpression booleanExpression1(y=true)
    annotation (Placement(transformation(extent={{-618,-350},{-596,-328}})));
  Modelica.Fluid.Interfaces.FluidPort_a port_a1
    "Fluid connector a (positive design flow direction is from port_a to port_b)"
    annotation (Placement(transformation(extent={{48,174},{68,194}})));
  Modelica.Fluid.Interfaces.FluidPort_b port_b1
    "Fluid connector b (positive design flow direction is from port_a to port_b)"
    annotation (Placement(transformation(extent={{74,174},{94,194}})));
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
  connect(acLoad.terminal, gri.terminal) annotation (Line(points={{336,-386},{
          336,-444},{18,-444},{18,-430}},                     color={0,120,120}));
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
  connect(domHotWatTan.port_aDom, souCol1.ports[1]) annotation (Line(points={{136,
          -284.4},{116,-284.4},{116,-249},{96,-249}},
        color={0,127,255}));
  connect(souCol1.ports[2], theMixVal.port_col) annotation (Line(points={{96,-247},
          {166,-247},{166,-248},{192,-248},{192,-260},{268,-260},{268,-272.4},{
          280,-272.4}},
                    color={0,127,255}));
  connect(theMixVal.port_hot, domHotWatTan.port_bDom) annotation (Line(points={{
          280,-265.2},{276,-265.2},{276,-284.4},{208,-284.4}}, color={0,127,255}));
  connect(theMixVal.TMixSet, conTSetMix.y) annotation (Line(points={{278,-254.4},
          {196,-254.4},{196,-79},{214.9,-79}}, color={0,0,127}));
  connect(sch.y[1], theMixVal.yMixSet) annotation (Line(points={{212.8,-114},{212.8,
          -116},{204,-116},{204,-243.6},{278,-243.6}}, color={0,0,127}));
  connect(weaDat1.weaBus, sinZonFlo1.weaBus) annotation (Line(
      points={{188,112},{160,112},{160,29},{216.8,29}},
      color={255,204,51},
      thickness=0.5));
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
  connect(buiHea1.ports_bHeaWat[1], tan.fluPorVol1[8]) annotation (Line(points={{296,
          18.9333},{308,18.9333},{308,-52},{42,-52},{42,-46},{41.46,-46},{41.46,
          -3.68}},                         color={0,127,255}));
  connect(wolfCGB20_GC1.port_b, val.port_a) annotation (Line(points={{-369.947,
          16.54},{-369.947,16},{-370,16},{-370,26},{-278,26}},
                                             color={0,127,255}));
  connect(val.y, y.y)
    annotation (Line(points={{-268,38},{-268,92},{-299,92}}, color={0,0,127}));
  connect(eleHea.TSet,conTSetHot1. y) annotation (Line(points={{-320,-291.8},{
          -324,-291.8},{-324,-290},{-352,-290},{-352,-283},{-375.1,-283}},
        color={0,0,127}));
  connect(eleHea.port_b, tan.fluPorVol[1]) annotation (Line(points={{-254,-315},
          {-100,-315},{-100,6},{-41.5,6},{-41.5,-21.98}},
                                                        color={0,127,255}));
  connect(eleHea.port_a, tan.fluPorVol[10]) annotation (Line(points={{-314,-315},
          {-340,-315},{-340,-360},{-88,-360},{-88,-28},{-41.5,-28},{-41.5,-0.02}},
        color={0,127,255}));
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
  connect(tan.T8, generatorControl.LowTlayer) annotation (Line(points={{54.27,
          -39.67},{72,-39.67},{72,-80},{56,-80},{56,-84},{4,-84},{4,-88},{-84,
          -88},{-84,-92},{-676,-92},{-676,-27.86},{-654.36,-27.86}}, color={0,0,
          127}));
  connect(wolfCGB20_GC1.port_a, tan.fluPorVol[2]) annotation (Line(points={{
          -368.773,-1.44},{-100,-1.44},{-100,8},{-41.5,8},{-41.5,-19.54}},
        color={0,127,255}));
  connect(buiHea1.ports_aHeaWat[1], tan.fluPorVol1[2]) annotation (Line(points={{190,
          18.9333},{180,18.9333},{180,-88},{41.46,-88},{41.46,-18.32}},
        color={0,127,255}));
  connect(sinZonFlo1.ports[2], buiHea1.ports_bHeaWat[2]) annotation (Line(
        points={{219.3,-8.88178e-16},{219.3,-52},{308,-52},{308,22.6667},{296,
          22.6667}}, color={0,127,255}));
  connect(sinZonFlo1.ports[1], buiHea1.ports_aHeaWat[2]) annotation (Line(
        points={{218.3,-8.88178e-16},{218.3,-36},{180,-36},{180,22.6667},{190,
          22.6667}},
        color={0,127,255}));
  connect(elHeaterControl.ON, eleHea.on) annotation (Line(points={{-500.48,-332},
          {-344,-332},{-344,-306.3},{-320,-306.3}}, color={255,0,255}));
  connect(booleanExpression1.y, elHeaterControl.Use) annotation (Line(points={{
          -594.9,-339},{-594.9,-337.76},{-568.96,-337.76}}, color={255,0,255}));
  connect(elHeaterControl.Tlayer5, tan.T5) annotation (Line(points={{-568.96,
          -321.76},{-584,-321.76},{-584,-264},{64,-264},{64,-72},{68,-72},{68,
          -36},{72,-36},{72,-12.83},{54.27,-12.83}}, color={0,0,127}));
  connect(port_a1, tan.fluPorVol1[2]) annotation (Line(points={{58,184},{58,-14},
          {41.46,-14},{41.46,-18.32}},
                                    color={0,127,255}));
  connect(port_b1, tan.fluPorVol1[4]) annotation (Line(points={{84,184},{84,-44},
          {26,-44},{26,-36},{41.46,-36},{41.46,-13.44}},
                                                  color={0,127,255}));
  annotation (
    Icon(
      coordinateSystem(
        preserveAspectRatio=false, extent={{-460,-480},{380,120}}), graphics={
          Bitmap(extent={{-402,-420},{372,226}}, fileName=
              "modelica://ProsNet/../thesis/report 7/New folder/sf4.JPG"),
        Rectangle(
          extent={{-456,-60},{380,-470}},
          lineColor={0,0,0},
          lineThickness=1),
        Polygon(
          points={{380,-62},{52,120},{-126,118},{-454,-60},{380,-62}},
          lineColor={0,0,0},
          lineThickness=1)}),
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
