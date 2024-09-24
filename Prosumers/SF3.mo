within ProsNet.Prosumers;
model SF3 "Example model of a building with loads provided as time series and
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
    T_start=324.15)
    annotation (Placement(transformation(extent={{-72,-72},{50,50}})));
  Fluid.Sources.Boundary_pT souCol1(redeclare package Medium = Media.Water,
      nPorts=3)
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
    annotation (Placement(transformation(extent={{-378,-208},{-312,-146}})));

  Components.BoundryCondition.ReaderTMY3 weaDat1(filNam=
        ModelicaServices.ExternalReferences.loadResource("modelica://ProsNet/Data/weatherdata/USA_CA_San.Francisco.Intl.AP.724940_TMY3.mos"))
    annotation (Placement(transformation(extent={{172,140},{192,160}})));
  Components.Generators.Digital_Twins.WolfBWS1_10_GC wolfBWS1_10_GC2
    annotation (Placement(transformation(extent={{-518,-28},{-438,52}})));
  Fluid.Valves.TwoWayEqualPercentage val(
    redeclare package Medium = Media.Water,
    m_flow_nominal=10,
    dpValve_nominal=6000)
    annotation (Placement(transformation(extent={{-306,-14},{-286,6}})));
  Modelica.Blocks.Sources.Ramp y(
    height=1,
    duration=1,
    offset=0) "Control signal"
    annotation (Placement(transformation(extent={{-320,74},{-300,94}})));
  Modelica.Blocks.Sources.RealExpression realExpression(y=1)
    annotation (Placement(transformation(extent={{-592,-40},{-572,-20}})));
  Fluid.Pumps.FlowControlled_m_flow pump(m_flow_nominal=10, addPowerToMedium=
        true)
    annotation (Placement(transformation(extent={{-232,-180},{-188,-146}})));
  Components.Electrical.Inductive acLoad2(linearized=false, mode=ProsNet.Fluid.Building_Fluid.Utili.Electrical.Types.Load.VariableZ_P_input)
    annotation (Placement(transformation(
        extent={{-22,-13},{22,13}},
        rotation=90,
        origin={-83,-398})));
  Modelica.Blocks.Sources.BooleanExpression booleanExpression(y=true)
    annotation (Placement(transformation(extent={{-764,2},{-742,24}})));
  Controls.GeneratorControlPackage.ControlGenerator.HPControl hPControl(TStart=
        323.65, TStop=324.15)
    annotation (Placement(transformation(extent={{-696,-22},{-612,62}})));
  Controls.GeneratorControlPackage.ControlGenerator.SolarThermalControl
    solarThermalControl(deltaTonST=3, TCollectorST=343)
    annotation (Placement(transformation(extent={{-544,-204},{-480,-140}})));
  Controls.BooleanToReal booToRea(realTrue=2.5, realFalse=0)
    annotation (Placement(transformation(extent={{-246,-130},{-226,-110}})));
  Modelica.Blocks.Sources.BooleanExpression booleanExpression1(y=true)
    annotation (Placement(transformation(extent={{-604,-190},{-582,-168}})));
  Modelica.Fluid.Interfaces.FluidPort_a port_a1
    "Fluid connector a (positive design flow direction is from port_a to port_b)"
    annotation (Placement(transformation(extent={{-308,58},{-112,212}}),
        iconTransformation(extent={{-308,58},{-112,212}})));
  Modelica.Fluid.Interfaces.FluidPort_b port_b1
    "Fluid connector b (positive design flow direction is from port_a to port_b)"
    annotation (Placement(transformation(extent={{-66,158},{44,238}}),
        iconTransformation(extent={{-66,158},{44,238}})));
  Components.Consumers.BuildingRCZ1Valve buildingRCZ1Valve(
    facMul=35,
    nZon=3,
    nPorts_aHeaWat=1,
    nPorts_aChiWat=1,
    nPorts_bHeaWat=1,
    nPorts_bChiWat=1)
    annotation (Placement(transformation(extent={{190,-34},{274,52}})));
equation

  connect(con.SOC, bat.SOC) annotation (Line(points={{-76,-570},{-76,-572},{
          -192,-572},{-192,-542.8},{-207.2,-542.8}}, color={0,0,127}));
  connect(con.y, bat.P) annotation (Line(points={{-41,-570},{-41,-572},{-32,
          -572},{-32,-508},{-260,-508},{-260,-526}}, color={0,0,127}));
  connect(conv.terminal_p, bat.terminal) annotation (Line(points={{158,-504},{
          172,-504},{172,-596},{-308,-596},{-308,-568}}, color={0,0,255}));
  connect(gri.terminal, conv.terminal_n) annotation (Line(points={{18,-430},{18,
          -444},{60,-444},{60,-504},{70,-504}}, color={0,120,120}));
  connect(pv.terminal, conv.terminal_p) annotation (Line(points={{436,-328},{
          432,-328},{432,-504},{158,-504}}, color={0,0,255}));
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
  connect(domHotWatTan.port_aDom, souCol1.ports[1]) annotation (Line(points={{136,
          -284.4},{92,-284.4},{92,-212},{140,-212},{140,-135.333},{114,-135.333}},
        color={0,127,255}));
  connect(souCol1.ports[2], theMixVal.port_col) annotation (Line(points={{114,
          -134},{140,-134},{140,-212},{92,-212},{92,-284},{132,-284},{132,
          -272.4},{280,-272.4}},
                    color={0,127,255}));
  connect(theMixVal.port_hot, domHotWatTan.port_bDom) annotation (Line(points={{
          280,-265.2},{276,-265.2},{276,-284.4},{208,-284.4}}, color={0,127,255}));
  connect(theMixVal.TMixSet, conTSetMix.y) annotation (Line(points={{278,-254.4},
          {196,-254.4},{196,-79},{214.9,-79}}, color={0,0,127}));
  connect(sch.y[1], theMixVal.yMixSet) annotation (Line(points={{212.8,-114},{212.8,
          -116},{204,-116},{204,-243.6},{278,-243.6}}, color={0,0,127}));
  connect(solCol.port_a, tan.fluPorVol[10]) annotation (Line(points={{-378,-177},
          {-388,-177},{-388,-220},{-41.5,-220},{-41.5,-0.02}},
                   color={0,127,255}));
  connect(weaDat1.weaBus, pv.weaBus) annotation (Line(
      points={{192,150},{204,150},{204,108},{469,108},{469,-295.6}},
      color={255,204,51},
      thickness=0.5));

  connect(weaDat1.weaBus, solCol.weaBus) annotation (Line(
      points={{192,150},{204,150},{204,100},{180,100},{180,68},{-378,68},{-378,
          -147.24}},
      color={255,204,51},
      thickness=0.5));
  connect(wolfBWS1_10_GC2.term_p, gri.terminal) annotation (Line(points={{
          -393.2,-7.2},{-393.2,-448},{18,-448},{18,-430}}, color={0,120,120}));
  connect(y.y, val.y)
    annotation (Line(points={{-299,84},{-296,84},{-296,8}}, color={0,0,127}));
  connect(wolfBWS1_10_GC2.port_b, val.port_a) annotation (Line(points={{-409.2,
          31.2},{-409.2,36},{-316,36},{-316,-4},{-306,-4}}, color={0,127,255}));
  connect(val.port_b, tan.fluPorVol[8]) annotation (Line(points={{-286,-4},{-84,
          -4},{-84,-14},{-60,-14},{-60,-12},{-41.5,-12},{-41.5,-4.9}},
                                                                   color={0,127,
          255}));
  connect(wolfBWS1_10_GC2.weaBus, weaDat1.weaBus) annotation (Line(
      points={{-593.2,40.8},{-593.2,128},{204,128},{204,150},{192,150}},
      color={255,204,51},
      thickness=0.5));
  connect(realExpression.y, wolfBWS1_10_GC2.HPAuxModulation) annotation (Line(
        points={{-571,-30},{-571,-32},{-532,-32},{-532,-10.8},{-516.8,-10.8}},
        color={0,0,127}));
  connect(solCol.port_b, pump.port_a) annotation (Line(points={{-312,-177},{
          -244,-177},{-244,-163},{-232,-163}}, color={0,127,255}));
  connect(pump.port_b, tan.fluPorVol[1]) annotation (Line(points={{-188,-163},{
          -41.5,-163},{-41.5,-21.98}}, color={0,127,255}));
  connect(acLoad2.Pow, pump.P) annotation (Line(points={{-83,-376},{-83,-147.7},
          {-185.8,-147.7}}, color={0,0,127}));
  connect(acLoad2.terminal, gri.terminal) annotation (Line(points={{-83,-420},{
          -83,-448},{18,-448},{18,-430}}, color={0,120,120}));
  connect(wolfBWS1_10_GC2.port_a, tan.fluPorVol[1]) annotation (Line(points={{
          -407.6,6.4},{-426,6.4},{-426,16},{-322,16},{-322,10},{-41.5,10},{
          -41.5,-21.98}}, color={0,127,255}));
  connect(hPControl.LowTlayer, tan.T8) annotation (Line(points={{-707.76,-12.76},
          {-740,-12.76},{-740,-88},{72,-88},{72,-39.67},{54.27,-39.67}}, color=
          {0,0,127}));
  connect(hPControl.Use, booleanExpression.y) annotation (Line(points={{-707.76,
          12.44},{-724.33,12.44},{-724.33,13},{-740.9,13}},
                                                        color={255,0,255}));
  connect(hPControl.TopTlayer, tan.T3) annotation (Line(points={{-707.76,33.44},
          {-724,33.44},{-724,80},{-332,80},{-332,64},{72,64},{72,3.64},{53.66,
          3.64}}, color={0,0,127}));
  connect(hPControl.ON, wolfBWS1_10_GC2.HPOn) annotation (Line(points={{-617.88,
          20},{-536,20},{-536,29.2},{-516.8,29.2}}, color={255,0,255}));
  connect(booToRea.y, pump.m_flow_in) annotation (Line(points={{-224,-120},{
          -224,-132},{-210,-132},{-210,-142.6}}, color={0,0,127}));
  connect(booToRea.u, solarThermalControl.ON) annotation (Line(points={{-248,
          -120},{-426,-120},{-426,-172},{-484.48,-172}}, color={255,0,255}));
  connect(solarThermalControl.Tlayer5, tan.T5) annotation (Line(points={{
          -552.96,-161.76},{-568,-161.76},{-568,-116},{-260,-116},{-260,-100},{
          60,-100},{60,-92},{76,-92},{76,-12.83},{54.27,-12.83}}, color={0,0,
          127}));
  connect(booleanExpression1.y, solarThermalControl.Use) annotation (Line(
        points={{-580.9,-179},{-580.9,-177.76},{-552.96,-177.76}}, color={255,0,
          255}));
  connect(port_a1, tan.fluPorVol1[2]) annotation (Line(points={{-210,135},{-210,
          14},{41.46,14},{41.46,-18.32}},
                                    color={0,127,255}));
  connect(port_b1, tan.fluPorVol1[4]) annotation (Line(points={{-11,198},{-11,
          64},{92,64},{92,-64},{88,-64},{88,-96},{41.46,-96},{41.46,-13.44}},
                                                  color={0,127,255}));
  connect(weaDat1.weaBus, buildingRCZ1Valve.weaBus) annotation (Line(
      points={{192,150},{204,150},{204,108},{232.14,108},{232.14,39.6733}},
      color={255,204,51},
      thickness=0.5));
  connect(buildingRCZ1Valve.ports_aHeaWat[1], tan.fluPorVol1[2]) annotation (
      Line(points={{190,0.4},{92,0.4},{92,-64},{88,-64},{88,-96},{41.46,-96},{
          41.46,-18.32}}, color={0,127,255}));
  connect(buildingRCZ1Valve.ports_bHeaWat[1], tan.fluPorVol1[8]) annotation (
      Line(points={{274,0.4},{288,0.4},{288,-48},{152,-48},{152,0},{92,0},{92,
          -64},{88,-64},{88,-96},{41.46,-96},{41.46,-3.68}}, color={0,127,255}));
  connect(buildingRCZ1Valve.ports_aChiWat[1], souCol1.ports[3]) annotation (
      Line(points={{190,-16.8},{140,-16.8},{140,-132.667},{114,-132.667}},
        color={0,127,255}));
  connect(buildingRCZ1Valve.ports_bChiWat[1], tan.fluPorVol1[8]) annotation (
      Line(points={{274,-16.8},{288,-16.8},{288,-48},{152,-48},{152,0},{92,0},{
          92,-64},{88,-64},{88,-96},{41.46,-96},{41.46,-3.68}}, color={0,127,
          255}));
  annotation (
    Icon(
      coordinateSystem(
        preserveAspectRatio=false, extent={{-460,-480},{380,120}}), graphics={
        Rectangle(
          extent={{-456,-58},{380,-468}},
          lineColor={0,0,0},
          lineThickness=1),
        Polygon(
          points={{380,-58},{52,124},{-126,122},{-454,-56},{380,-58}},
          lineColor={0,0,0},
          lineThickness=1),
        Text(
          extent={{-438,-48},{378,-464}},
          textColor={28,108,200},
          textString="SF3")}),
    Diagram(
        coordinateSystem(
        preserveAspectRatio=false, extent={{-460,-480},{380,120}})),
    __Dymola_Commands(
      file="modelica://Buildings/Resources/Scripts/Dymola/DHC/Loads/Heating/Examples/BuildingTimeSeriesWithETS.mos" "Simulate and plot"),
    experiment(
      StopTime=40000,
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
end SF3;
