within ProsNet.Fluid.Pumps.Examples;
model TestDHW_Storage_Solar
  "Example model for storage tank with external heat exchanger"
  extends Modelica.Icons.Example;
  package Medium = Buildings.Media.Water "Medium model";
  parameter Modelica.Units.SI.Temperature TCol = 273.15+10 "Temperature of domestic cold water supply";
  parameter Modelica.Units.SI.MassFlowRate mHea_flow_nominal = datWatHea.QHex_flow_nominal/4200/(55 - 50) "Tank heater water loop nominal mass flow";
  parameter Buildings.DHC.Loads.HotWater.Data.GenericDomesticHotWaterWithHeatExchanger
    datWatHea(VTan=0.1892706, mDom_flow_nominal=6.52944E-06*1000)
    "Data for heat pump water heater with tank"
    annotation (Placement(transformation(extent={{-94,-30},{-74,-10}})));

  Buildings.DHC.Loads.HotWater.StorageTankWithExternalHeatExchanger
    domHotWatTan(redeclare package MediumDom = Medium, redeclare package
      MediumHea = Medium,
    dat=datWatHea) "Storage tank with external heat exchanger"
    annotation (Placement(transformation(extent={{0,20},{20,40}})));
  Buildings.Controls.OBC.CDL.Conversions.BooleanToReal booToRea(realTrue=
        mHea_flow_nominal)
    annotation (Placement(transformation(extent={{-3,-3},{3,3}},
        rotation=180,
        origin={-17,-37})));
  Modelica.Blocks.Sources.Constant conTSetHot(k(
      final unit="K",
      displayUnit="degC") = 313.15)
    "Temperature setpoint for hot water supply to fixture"
    annotation (Placement(transformation(extent={{-84,20},{-64,40}})));
  Buildings.Fluid.SolarCollectors.ASHRAE93 solCol(
    redeclare package Medium = Buildings.Media.Water,
    shaCoe=0,
    rho=0.2,
    nColType=Buildings.Fluid.SolarCollectors.Types.NumberSelection.Area,
    totalArea=1.312,
    sysConfig=Buildings.Fluid.SolarCollectors.Types.SystemConfiguration.Series,
    per=Buildings.Fluid.SolarCollectors.Data.GlazedFlatPlate.FP_GuangdongFSPTY95(),
    nPanels=1,
    nSeg=9,
    azi=0.010646508437165,
    til=0.054803338512622,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
    massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState)
    "Flat plate solar collector model"
    annotation (Placement(transformation(extent={{100,-114},{138,-84}})));

  Test_prosumer1.Test_Pump_controler testClosedLoop_Pump
    annotation (Placement(transformation(extent={{-20,-76},{-2,-54}})));
  Buildings.BoundaryConditions.WeatherData.ReaderTMY3 weaDat(filNam=
        ModelicaServices.ExternalReferences.loadResource(
        "modelica://Buildings/Resources/weatherdata/USA_CA_San.Francisco.Intl.AP.724940_TMY3.mos"))
    annotation (Placement(transformation(extent={{66,110},{86,130}})));
  Buildings.Fluid.Sources.Boundary_pT souCol(
    nPorts=2,
    redeclare package Medium = Medium,
    T(displayUnit="degC") = 283.15) "Source of domestic cold water"
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-40,-20})));
  Modelica.Blocks.Sources.CombiTimeTable sch(
    tableOnFile=true,
    tableName="tab1",
    fileName=Modelica.Utilities.Files.loadResource(
        "modelica://Buildings/Resources/Data/DHC/Loads/HotWater/DHW_ApartmentMidRise.mos"),
    smoothness=Modelica.Blocks.Types.Smoothness.ContinuousDerivative,
    extrapolation=Modelica.Blocks.Types.Extrapolation.Periodic)
    "Domestic hot water fixture draw fraction schedule"
    annotation (Placement(transformation(extent={{-68,84},{-48,104}})));

  Buildings.DHC.Loads.HotWater.ThermostaticMixingValve theMixVal(redeclare
      package Medium = Medium, mMix_flow_nominal=1.2*datWatHea.mDom_flow_nominal)
    annotation (Placement(transformation(extent={{52,76},{72,96}})));
  Modelica.Blocks.Sources.Constant conTSetMix(k(
      final unit="K",
      displayUnit="degC") = 308.15)
    "Temperature setpoint for mixed water supply to fixture"
    annotation (Placement(transformation(extent={{-86,50},{-66,70}})));
equation
  connect(booToRea.u, domHotWatTan.charge) annotation (Line(points={{-13.4,
          -37},{-13.4,-38},{22,-38},{22,21}},
                                 color={255,0,255}));
  connect(domHotWatTan.TDomSet, conTSetHot.y) annotation (Line(points={{-1,30},
          {-63,30}},                  color={0,0,127}));
  connect(solCol.port_a,testClosedLoop_Pump. port_a) annotation (Line(points={{100,-99},
          {100,-100},{-26,-100},{-26,-64.78},{-19.64,-64.78}},       color={0,
          127,255}));
  connect(domHotWatTan.port_bHea, testClosedLoop_Pump.port_b) annotation (
      Line(points={{0,24},{0,-64.78},{-1.64,-64.78}}, color={0,127,255}));
  connect(solCol.port_b, domHotWatTan.port_aHea) annotation (Line(points={{
          138,-99},{138,-100},{142,-100},{142,24},{20,24}}, color={0,127,255}));
  connect(booToRea.y, testClosedLoop_Pump.SetInSignal) annotation (Line(
        points={{-20.6,-37},{-20.6,-52.9},{-18.38,-52.9}}, color={0,0,127}));
  connect(solCol.weaBus, weaDat.weaBus) annotation (Line(
      points={{100,-84.6},{100,120},{86,120}},
      color={255,204,51},
      thickness=0.5));
  connect(souCol.ports[1], domHotWatTan.port_aDom) annotation (Line(points={{
          -39,-10},{-39,28},{-42,28},{-42,36},{0,36}}, color={0,127,255}));
  connect(theMixVal.yMixSet,sch. y[1]) annotation (Line(points={{51,94},{-47,
          94}},               color={0,0,127}));
  connect(theMixVal.port_hot, domHotWatTan.port_bDom) annotation (Line(points
        ={{52,82},{26,82},{26,36},{20,36}}, color={0,127,255}));
  connect(theMixVal.port_col, souCol.ports[2]) annotation (Line(points={{52,
          78},{-38,78},{-38,36},{-42,36},{-42,28},{-41,28},{-41,-10}}, color=
          {0,127,255}));
  connect(conTSetMix.y, theMixVal.TMixSet) annotation (Line(points={{-65,60},
          {-40,60},{-40,88},{51,88}}, color={0,0,127}));
  annotation (
          __Dymola_Commands(
      file="modelica://Buildings/Resources/Scripts/Dymola/DHC/Loads/HotWater/Examples/StorageTankWithExternalHeatExchanger.mos" "Simulate and plot"),
Documentation(info="<html>
<p>
Example model of a fresh water station that heats up domestic hot water.
Input is a load profile which is sent to a model that computes the hot and cold water draw.
If the tank needs to be recharged, then tank water is circulated through a heater
with a prescribed temperature lift.
</p>
</html>", revisions="<html>
<ul>
<li>
November 15, 2023, by David Blum:<br/>
Add heater with constant dT as heat source.
</li>
<li>
October 5, 2023, by Michael Wetter:<br/>
First implementation.
</li>
</ul>
</html>"),
    experiment(
      StopTime=172800,
      Tolerance=1e-06),
    Diagram(coordinateSystem(extent={{-100,-120},{140,100}})),
    Icon(coordinateSystem(extent={{-100,-120},{140,100}})));
end TestDHW_Storage_Solar;
