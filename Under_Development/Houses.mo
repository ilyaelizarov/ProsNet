within ProsNet.Under_Development;
package Houses
  model SF1 "Example model of a building with loads provided as time series and
  connected to an ETS for cooling"
    extends Modelica.Icons.Example;
    package Medium=Buildings.Media.Water
      "Medium model";
    Buildings.Electrical.DC.Sources.PVSimpleOriented
                                           pv(
      A=200e3/800/0.12,
      til=0.34906585039887,
      azi=-0.78539816339745,
      V_nominal=480) "PV array"
      annotation (Placement(transformation(extent={{414,-432},{346,-362}})));
    Buildings.Electrical.DC.Storage.Battery bat(EMax=500e3*4*3600, V_nominal=
          480)       "Battery"
      annotation (Placement(transformation(extent={{-124,-500},{-174,-454}})));
    Buildings.Electrical.AC.OnePhase.Conversion.ACDCConverter conv(
        conversionFactor=480/480, eta=0.9)
               "AC/DC converter"
      annotation (Placement(transformation(extent={{12,-474},{-30,-428}})));
    Buildings.Electrical.AC.OnePhase.Sources.Grid gri(
      f=60,
      V=480,
      phiSou=0) annotation (Placement(transformation(extent={{62,-410},{102,
              -370}})));
    Buildings.Examples.ChillerPlant.BaseClasses.Controls.BatteryControl
                                        con "Battery controller"
      annotation (Placement(transformation(extent={{-212,-444},{-192,-424}})));
    Buildings.Electrical.AC.OnePhase.Loads.Inductive acLoad(mode=Buildings.Electrical.Types.Load.VariableZ_P_input,
        V_nominal=480)
      annotation (Placement(transformation(extent={{22,-20},{-22,20}},
          rotation=270,
          origin={300,-350})));
    Buildings.BoundaryConditions.WeatherData.ReaderTMY3 weaDat(filNam=
          ModelicaServices.ExternalReferences.loadResource(
          "modelica://Buildings/Resources/weatherdata/USA_CA_San.Francisco.Intl.AP.724940_TMY3.mos"))
      annotation (Placement(transformation(extent={{210,110},{230,130}})));
    Modelica.Blocks.Sources.RealExpression realExpression(y=1)
      annotation (Placement(transformation(extent={{-532,6},{-512,26}})));
    Modelica.Blocks.Sources.RealExpression realExpression1(y=1)
      annotation (Placement(transformation(extent={{-558,-120},{-538,-100}})));
    Modelica.Blocks.Sources.BooleanExpression booleanExpression(y=true)
      annotation (Placement(transformation(extent={{-562,-98},{-542,-78}})));
    Buildings.Fluid.Actuators.Valves.ThreeWayLinear valLin(
      redeclare package Medium = Buildings.Media.Water,
      l={0.05,0.05},
      m_flow_nominal=2,
      use_inputFilter=false,
      dpValve_nominal=6000,
      energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial)
      "Valve model, linear opening characteristics"
      annotation (Placement(transformation(extent={{-172,-148},{-152,-128}})));
    Buildings.Fluid.Actuators.Valves.ThreeWayLinear valLin1(
      redeclare package Medium = Buildings.Media.Water,
      l={0.05,0.05},
      m_flow_nominal=2,
      use_inputFilter=false,
      dpValve_nominal=6000,
      energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial)
      "Valve model, linear opening characteristics"
      annotation (Placement(transformation(extent={{-244,-26},{-224,-6}})));
    Modelica.Blocks.Sources.Ramp y(
      height=1,
      duration=1,
      offset=0) "Control signal"
      annotation (Placement(transformation(extent={{-290,56},{-270,76}})));
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
      annotation (Placement(transformation(extent={{-334,-316},{-274,-262}})));

    parameter Buildings.DHC.Loads.HotWater.Data.GenericDomesticHotWaterWithHeatExchanger
      datWatHea(VTan=0.1892706, mDom_flow_nominal=6.52944E-06*1000)
      "Data for heat pump water heater with tank"
      annotation (Placement(transformation(extent={{438,-224},{458,-204}})));
    Buildings.DHC.Loads.HotWater.StorageTankWithExternalHeatExchanger DHWTan(
      redeclare package MediumDom = Medium,
      redeclare package MediumHea = Medium,
      dat=datWatHea) "Storage tank with external heat exchanger"
      annotation (Placement(transformation(extent={{160,-286},{222,-222}})));
    Buildings.Controls.OBC.CDL.Conversions.BooleanToReal booToRea(realTrue=0.5)
      annotation (Placement(transformation(extent={{-8,-8},{8,8}},
          rotation=180,
          origin={20,-218})));
    Modelica.Blocks.Sources.Constant conTSetHot(k(
        final unit="K",
        displayUnit="degC") = 313.15)
      "Temperature setpoint for hot water supply to fixture"
      annotation (Placement(transformation(extent={{132,-258},{140,-250}})));
    Modelica.Blocks.Sources.CombiTimeTable sch(
      tableOnFile=true,
      tableName="tab1",
      fileName=Modelica.Utilities.Files.loadResource(
          "modelica://Buildings/Resources/Data/DHC/Loads/HotWater/DHW_ApartmentMidRise.mos"),
      smoothness=Modelica.Blocks.Types.Smoothness.ContinuousDerivative,
      extrapolation=Modelica.Blocks.Types.Extrapolation.Periodic)
      "Domestic hot water fixture draw fraction schedule"
      annotation (Placement(transformation(extent={{-9,-9},{9,9}},
          rotation=180,
          origin={449,-175})));

    Buildings.DHC.Loads.HotWater.ThermostaticMixingValve theMixVal(redeclare
        package Medium = Medium, mMix_flow_nominal=1.2*datWatHea.mDom_flow_nominal)
      annotation (Placement(transformation(extent={{254,-218},{284,-190}})));
    Modelica.Blocks.Sources.Constant conTSetMix(k(
        final unit="K",
        displayUnit="degC") = 308.15)
      "Temperature setpoint for mixed water supply to fixture"
      annotation (Placement(transformation(extent={{-8,-8},{8,8}},
          rotation=180,
          origin={448,-148})));
    Buildings.Electrical.AC.OnePhase.Loads.Inductive acLoad1(mode=Buildings.Electrical.Types.Load.VariableZ_P_input,
        V_nominal=480)
      annotation (Placement(transformation(extent={{23,-22},{-23,22}},
          rotation=270,
          origin={250,-349})));
    Buildings.Electrical.DC.Lines.TwoPortResistance lin(R=0.05)
      "Transmission line"
      annotation (Placement(transformation(extent={{-25,-17},{25,17}},
          rotation=0,
          origin={79,-495})));
    Buildings.Fluid.Sources.Boundary_pT souCol(
      nPorts=1,
      redeclare package Medium = Medium,
      T(displayUnit="degC"))          "Source of domestic cold water"
      annotation (Placement(transformation(extent={{-12,-12},{12,12}},
          rotation=0,
          origin={204,-188})));
    Buildings.DHC.Loads.BaseClasses.BuildingTimeSeries buiHea(
      have_chiWat=false,
      filNam=
          "modelica://Buildings/Resources/Data/DHC/Loads/Examples/SwissResidential_20190916.mos",
      nPorts_aChiWat=1,
      nPorts_bChiWat=1,
      nPorts_bHeaWat=2,
      nPorts_aHeaWat=2)
      "Building with heating only"
      annotation (Placement(transformation(extent={{174,-84},{248,-20}})));

    Buildings.Fluid.Actuators.Valves.ThreeWayLinear valLin2(
      redeclare package Medium = Buildings.Media.Water,
      l={0.05,0.05},
      m_flow_nominal=2,
      use_inputFilter=false,
      dpValve_nominal=6000,
      energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial)
      "Valve model, linear opening characteristics"
      annotation (Placement(transformation(extent={{-10,-10},{10,10}},
          rotation=180,
          origin={120,-124})));
    Modelica.Blocks.Sources.Ramp y1(
      height=1,
      duration=1,
      offset=0) "Control signal"
      annotation (Placement(transformation(extent={{98,10},{118,30}})));
    Buildings.ThermalZones.Detailed.Validation.BaseClasses.SingleZoneFloor sinZonFlo(
      redeclare package Medium =
          Modelica.Media.Water.ConstantPropertyLiquidWater,
      use_windPressure=false,
      gai(K=0*[0.4; 0.4; 0.2]))
      "Single-zone floor model"
      annotation (Placement(transformation(extent={{200,-70},{220,-50}})));
    Generators.Digital_Twins.NeoTower2_GC neoTower2_GC
      annotation (Placement(transformation(extent={{-526,-174},{-416,-98}})));
    Generators.Digital_Twins.WolfCGB14_GC wolfCGB14_GC
      annotation (Placement(transformation(extent={{-448,-2},{-354,52}})));
    Storage.StratifiedHeatStorage tan(
      redeclare package Medium = Buildings.Media.Water,
      m_flow_nominal=0.1,
      VTan=775,
      hTan=3,
      dIns=0.3,
      nSeg=10)
      annotation (Placement(transformation(extent={{-60,-56},{16,20}})));
    Fluid.Pumps.Test_Pump_controler test_Pump_controler
      annotation (Placement(transformation(extent={{-42,-264},{-22,-244}})));
  equation
    connect(bat.SOC,con. SOC) annotation (Line(
        points={{-176.5,-463.2},{-220,-463.2},{-220,-434},{-213.25,-434}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(con.y,bat. P) annotation (Line(
        points={{-191.375,-434},{-191.375,-436},{-149,-436},{-149,-454}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(bat.terminal,conv. terminal_p) annotation (Line(
        points={{-124,-477},{-44,-477},{-44,-451},{-30,-451}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(acLoad.terminal,gri. terminal) annotation (Line(
        points={{300,-372},{300,-410},{82,-410}},
        color={0,120,120},
        smooth=Smooth.None));
    connect(conv.terminal_n,gri. terminal) annotation (Line(
        points={{12,-451},{82,-451},{82,-410}},
        color={0,120,120},
        smooth=Smooth.None));
    connect(pv.weaBus, weaDat.weaBus) annotation (Line(
        points={{380,-365.5},{380,120},{230,120}},
        color={255,204,51},
        thickness=0.5));
    connect(y.y, valLin1.y) annotation (Line(points={{-269,66},{-234,66},{-234,
            -4}}, color={0,0,127}));
    connect(y.y, valLin.y) annotation (Line(points={{-269,66},{-162,66},{-162,
            -126}},                color={0,0,127}));
    connect(solCol.weaBus, weaDat.weaBus) annotation (Line(
        points={{-334,-263.08},{-336,-263.08},{-336,120},{230,120}},
        color={255,204,51},
        thickness=0.5));
    connect(booToRea.u, DHWTan.charge) annotation (Line(points={{29.6,-218},{
            29.6,-220},{86,-220},{86,-296},{228.2,-296},{228.2,-282.8}}, color=
            {255,0,255}));
    connect(DHWTan.TDomSet, conTSetHot.y)
      annotation (Line(points={{156.9,-254},{140.4,-254}}, color={0,0,127}));
    connect(theMixVal.yMixSet,sch. y[1]) annotation (Line(points={{252.5,-192.8},
            {244,-192.8},{244,-175},{439.1,-175}},
                                color={0,0,127}));
    connect(conTSetMix.y, theMixVal.TMixSet) annotation (Line(points={{439.2,
            -148},{240,-148},{240,-201.2},{252.5,-201.2}}, color={0,0,127}));
    connect(solCol.port_b, DHWTan.port_aHea) annotation (Line(points={{-274,
            -289},{-274,-300},{244,-300},{244,-273.2},{222,-273.2}}, color={0,
            127,255}));
    connect(theMixVal.port_hot, DHWTan.port_bDom) annotation (Line(points={{254,
            -209.6},{240,-209.6},{240,-234.8},{222,-234.8}}, color={0,127,255}));
    connect(DHWTan.PEle, acLoad1.Pow) annotation (Line(points={{225.1,-254},{
            225.1,-256},{250,-256},{250,-326}}, color={0,0,127}));
    connect(acLoad1.terminal, gri.terminal) annotation (Line(points={{250,-372},
            {250,-410},{82,-410}}, color={0,120,120}));
    connect(lin.terminal_p, pv.terminal) annotation (Line(points={{104,-495},{
            428,-495},{428,-397},{414,-397}}, color={0,0,255}));
    connect(lin.terminal_n, conv.terminal_p) annotation (Line(points={{54,-495},
            {-48,-495},{-48,-476},{-44,-476},{-44,-451},{-30,-451}}, color={0,0,
            255}));
    connect(theMixVal.port_col, souCol.ports[1]) annotation (Line(points={{254,
            -215.2},{236,-215.2},{236,-188},{216,-188}}, color={0,127,255}));
    connect(buiHea.PPum, acLoad.Pow) annotation (Line(points={{250.467,-45.6},{
            290,-45.6},{290,-46},{300,-46},{300,-328}},
                                color={0,0,127}));
    connect(buiHea.ports_bHeaWat[1], valLin2.port_1) annotation (Line(points={{248,
            -59.4667},{260,-59.4667},{260,-124},{130,-124}},
                                                           color={0,127,255}));
    connect(DHWTan.port_aDom, valLin2.port_3) annotation (Line(points={{160,
            -234.8},{104,-234.8},{104,-114},{120,-114}}, color={0,127,255}));
    connect(y1.y, valLin2.y) annotation (Line(points={{119,20},{140,20},{140,
            -148},{120,-148},{120,-136}}, color={0,0,127}));
    connect(sinZonFlo.weaBus, weaDat.weaBus) annotation (Line(
        points={{203.4,-51.5},{164,-51.5},{164,120},{230,120}},
        color={255,204,51},
        thickness=0.5));
    connect(neoTower2_GC.weaBus, weaDat.weaBus) annotation (Line(
        points={{-457.938,-71.7455},{-457.938,120},{230,120}},
        color={255,204,51},
        thickness=0.5));
    connect(booleanExpression.y, neoTower2_GC.CHPOn) annotation (Line(points={{
            -541,-88},{-541,-91.4364},{-507.094,-91.4364}}, color={255,0,255}));
    connect(realExpression1.y, neoTower2_GC.CHPModulation) annotation (Line(
          points={{-537,-110},{-537,-114.236},{-507.094,-114.236}}, color={0,0,
            127}));
    connect(neoTower2_GC.port_b, valLin.port_3) annotation (Line(points={{-438,
            -122.873},{-192,-122.873},{-192,-148},{-162,-148}}, color={0,127,
            255}));
    connect(neoTower2_GC.port_a, valLin1.port_3) annotation (Line(points={{
            -436.625,-144.291},{-332,-144.291},{-332,-40},{-234,-40},{-234,-26}},
          color={0,127,255}));
    connect(neoTower2_GC.term_p, gri.terminal) annotation (Line(points={{
            -458.625,-158.109},{-458.625,-410},{82,-410}}, color={0,120,120}));
    connect(realExpression.y, wolfCGB14_GC.ControlIn) annotation (Line(points={
            {-511,16},{-460,16},{-460,17.8818},{-438.353,17.8818}}, color={0,0,
            127}));
    connect(wolfCGB14_GC.weaBus, weaDat.weaBus) annotation (Line(
        points={{-410.4,47.5818},{-410.4,120},{230,120}},
        color={255,204,51},
        thickness=0.5));
    connect(wolfCGB14_GC.port_b, valLin.port_1) annotation (Line(points={{
            -369.337,38.7455},{-369.337,60},{-304,60},{-304,-120},{-180,-120},{
            -180,-138},{-172,-138}}, color={0,127,255}));
    connect(wolfCGB14_GC.port_a, valLin1.port_1) annotation (Line(points={{
            -362.905,18.6182},{-256,18.6182},{-256,-16},{-244,-16}}, color={0,
            127,255}));
    connect(wolfCGB14_GC.term_p, gri.terminal) annotation (Line(points={{
            -398.526,-9.36364},{-398.526,-410},{82,-410}}, color={0,120,120}));
    connect(tan.port_a, valLin1.port_2) annotation (Line(points={{-22,20},{-24,
            20},{-24,32},{-212,32},{-212,-16},{-224,-16}}, color={0,127,255}));
    connect(valLin.port_2, tan.fluPorVol[2]) annotation (Line(points={{-152,
            -138},{-152,-140},{-72,-140},{-72,-23.32},{-41,-23.32}}, color={0,
            127,255}));
    connect(valLin2.port_2, tan.fluPorVol[4]) annotation (Line(points={{110,
            -124},{-64,-124},{-64,-20.28},{-41,-20.28}}, color={0,127,255}));
    connect(buiHea.ports_aHeaWat[1], tan.port_b) annotation (Line(points={{174,
            -59.4667},{28,-59.4667},{28,-68},{-22,-68},{-22,-56}}, color={0,127,
            255}));
    connect(sinZonFlo.ports[1], buiHea.ports_aHeaWat[2]) annotation (Line(
          points={{204.15,-66},{204.15,-57.3333},{174,-57.3333}}, color={0,127,
            255}));
    connect(sinZonFlo.ports[2], buiHea.ports_bHeaWat[2]) annotation (Line(
          points={{204.65,-66},{226,-66},{226,-57.3333},{248,-57.3333}}, color=
            {0,127,255}));
    connect(test_Pump_controler.port_a, solCol.port_a) annotation (Line(points=
            {{-41.6,-253.8},{-348,-253.8},{-348,-289},{-334,-289}}, color={0,
            127,255}));
    connect(test_Pump_controler.port_b, DHWTan.port_bHea) annotation (Line(
          points={{-21.6,-253.8},{120,-253.8},{120,-273.2},{160,-273.2}}, color
          ={0,127,255}));
    connect(test_Pump_controler.SetInSignal, booToRea.y) annotation (Line(
          points={{-40.2,-243},{-40.2,-218},{10.4,-218}}, color={0,0,127}));
    connect(test_Pump_controler.term_p, gri.terminal) annotation (Line(points={
            {-26.4,-267.4},{-26.4,-410},{82,-410}}, color={0,120,120}));
    annotation (
      Icon(
        coordinateSystem(
          preserveAspectRatio=false, extent={{-460,-500},{380,120}})),
      Diagram(
          coordinateSystem(
          preserveAspectRatio=false, extent={{-460,-500},{380,120}})),
      __Dymola_Commands(
        file="modelica://Buildings/Resources/Scripts/Dymola/DHC/Loads/Heating/Examples/BuildingTimeSeriesWithETS.mos" "Simulate and plot"),
      experiment(
        StartTime=12000,
        StopTime=90000,
        __Dymola_NumberOfIntervals=1000,
        Tolerance=1e-06,
        __Dymola_Algorithm="Dassl"),
      Documentation(info="<html>
<p>
This model provides an example for a building with loads provided
as time series and connected to a direct ETS for heating with the
return heating water temperature controlled below a maximum threshold.
</p>
</html>",   revisions="<html>
<ul>
<li>
March 20, 2022, by Chengnan Shi:<br/>
First implementation.
</li>
</ul>
</html>"));
  end SF1;

  model SF2 "Example model of a building with loads provided as time series and
  connected to an ETS for cooling"
    extends Modelica.Icons.Example;
    package Medium=Buildings.Media.Water
      "Medium model";
    Buildings.Electrical.DC.Sources.PVSimpleOriented
                                           pv(
      A=200e3/800/0.12,
      til=0.34906585039887,
      azi=-0.78539816339745,
      V_nominal=480) "PV array"
      annotation (Placement(transformation(extent={{414,-432},{346,-362}})));
    Buildings.Electrical.DC.Storage.Battery bat(EMax=500e3*4*3600, V_nominal=
          480)       "Battery"
      annotation (Placement(transformation(extent={{-124,-500},{-174,-454}})));
    Buildings.Electrical.AC.OnePhase.Conversion.ACDCConverter conv(
        conversionFactor=480/480, eta=0.9)
               "AC/DC converter"
      annotation (Placement(transformation(extent={{12,-474},{-30,-428}})));
    Buildings.Electrical.AC.OnePhase.Sources.Grid gri(
      f=60,
      V=480,
      phiSou=0) annotation (Placement(transformation(extent={{62,-410},{102,
              -370}})));
    Buildings.Examples.ChillerPlant.BaseClasses.Controls.BatteryControl
                                        con "Battery controller"
      annotation (Placement(transformation(extent={{-212,-444},{-192,-424}})));
    Buildings.Electrical.AC.OnePhase.Loads.Inductive acLoad(mode=Buildings.Electrical.Types.Load.VariableZ_P_input,
        V_nominal=480)
      annotation (Placement(transformation(extent={{22,-20},{-22,20}},
          rotation=270,
          origin={316,-350})));
    Buildings.BoundaryConditions.WeatherData.ReaderTMY3 weaDat(filNam=
          ModelicaServices.ExternalReferences.loadResource(
          "modelica://Buildings/Resources/weatherdata/USA_CA_San.Francisco.Intl.AP.724940_TMY3.mos"))
      annotation (Placement(transformation(extent={{210,110},{230,130}})));
    Modelica.Blocks.Sources.RealExpression realExpression(y=1)
      annotation (Placement(transformation(extent={{-514,-24},{-494,-4}})));
    Buildings.Fluid.Actuators.Valves.ThreeWayLinear valLin1(
      redeclare package Medium = Buildings.Media.Water,
      l={0.05,0.05},
      m_flow_nominal=2,
      use_inputFilter=false,
      dpValve_nominal=6000,
      energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial)
      "Valve model, linear opening characteristics"
      annotation (Placement(transformation(extent={{-130,-106},{-110,-86}})));
    Modelica.Blocks.Sources.Ramp y(
      height=1,
      duration=1,
      offset=0) "Control signal"
      annotation (Placement(transformation(extent={{-292,56},{-272,76}})));

    parameter Buildings.DHC.Loads.HotWater.Data.GenericDomesticHotWaterWithHeatExchanger
      datWatHea(VTan=0.1892706, mDom_flow_nominal=6.52944E-06*1000)
      "Data for heat pump water heater with tank"
      annotation (Placement(transformation(extent={{438,-224},{458,-204}})));
    Modelica.Blocks.Sources.Constant conTSetHot(k(
        final unit="K",
        displayUnit="degC") = 313.15)
      "Temperature setpoint for hot water supply to fixture"
      annotation (Placement(transformation(extent={{100,-266},{114,-252}})));
    Buildings.Fluid.Sources.Boundary_pT souCol(
      nPorts=2,
      redeclare package Medium = Medium,
      T(displayUnit="degC"))          "Source of domestic cold water"
      annotation (Placement(transformation(extent={{-17,-17},{17,17}},
          rotation=0,
          origin={109,-183})));
    Modelica.Blocks.Sources.CombiTimeTable sch(
      tableOnFile=true,
      tableName="tab1",
      fileName=Modelica.Utilities.Files.loadResource(
          "modelica://Buildings/Resources/Data/DHC/Loads/HotWater/DHW_ApartmentMidRise.mos"),
      smoothness=Modelica.Blocks.Types.Smoothness.ContinuousDerivative,
      extrapolation=Modelica.Blocks.Types.Extrapolation.Periodic)
      "Domestic hot water fixture draw fraction schedule"
      annotation (Placement(transformation(extent={{-12,-12},{12,12}},
          rotation=180,
          origin={450,-170})));

    Buildings.DHC.Loads.HotWater.ThermostaticMixingValve theMixVal(redeclare
        package Medium = Medium, mMix_flow_nominal=1.2*datWatHea.mDom_flow_nominal)
      annotation (Placement(transformation(extent={{248,-216},{292,-178}})));
    Modelica.Blocks.Sources.Constant conTSetMix(k(
        final unit="K",
        displayUnit="degC") = 308.15)
      "Temperature setpoint for mixed water supply to fixture"
      annotation (Placement(transformation(extent={{-11,-11},{11,11}},
          rotation=180,
          origin={451,-135})));
    Buildings.Electrical.AC.OnePhase.Loads.Inductive acLoad1(mode=Buildings.Electrical.Types.Load.VariableZ_P_input,
        V_nominal=480)
      annotation (Placement(transformation(extent={{23,-22},{-23,22}},
          rotation=270,
          origin={250,-349})));
    Buildings.Electrical.DC.Lines.TwoPortResistance lin(R=0.05)
      "Transmission line"
      annotation (Placement(transformation(extent={{-25,-17},{25,17}},
          rotation=0,
          origin={79,-489})));
    Modelica.Blocks.Sources.RealExpression realExpression1(y=10)
      annotation (Placement(transformation(extent={{-210,-128},{-190,-108}})));
    CoSES_ProHMo.Consumer.MyDHW domHotWat(
      redeclare package MediumDom = Buildings.Media.Water,
      redeclare package MediumHea = Buildings.Media.Water,
      dat=datWatHea)
      annotation (Placement(transformation(extent={{144,-304},{208,-232}})));
    Buildings.Fluid.Actuators.Valves.ThreeWayLinear valLin2(
      redeclare package Medium = Buildings.Media.Water,
      l={0.05,0.05},
      m_flow_nominal=2,
      use_inputFilter=false,
      dpValve_nominal=6000,
      energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial)
      "Valve model, linear opening characteristics"
      annotation (Placement(transformation(extent={{-186,-10},{-166,10}})));
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
      annotation (Placement(transformation(extent={{-338,-160},{-278,-106}})));

    Modelica.Blocks.Sources.RealExpression realExpression2(y=1)
      annotation (Placement(transformation(extent={{-488,-310},{-468,-290}})));
    Modelica.Blocks.Sources.RealExpression realExpression5(y=1)
      annotation (Placement(transformation(extent={{-490,-330},{-470,-310}})));
    Modelica.Blocks.Sources.BooleanExpression booleanExpression(y=true)
      annotation (Placement(transformation(extent={{-486,-280},{-466,-260}})));
    Modelica.Blocks.Sources.BooleanExpression booleanExpression1(y=true)
      annotation (Placement(transformation(extent={{-488,-260},{-468,-240}})));
    Buildings.Fluid.Actuators.Valves.ThreeWayLinear valLin3(
      redeclare package Medium = Buildings.Media.Water,
      l={0.05,0.05},
      m_flow_nominal=2,
      use_inputFilter=false,
      dpValve_nominal=6000,
      energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial)
      "Valve model, linear opening characteristics"
      annotation (Placement(transformation(extent={{-210,-284},{-190,-264}})));
    Buildings.Fluid.Actuators.Valves.ThreeWayLinear valLin4(
      redeclare package Medium = Buildings.Media.Water,
      l={0.05,0.05},
      m_flow_nominal=2,
      use_inputFilter=false,
      dpValve_nominal=6000,
      energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial)
      "Valve model, linear opening characteristics"
      annotation (Placement(transformation(extent={{-136,-320},{-116,-300}})));
    Buildings.Fluid.Actuators.Valves.ThreeWayLinear valLin6(
      redeclare package Medium = Buildings.Media.Water,
      l={0.05,0.05},
      m_flow_nominal=2,
      use_inputFilter=false,
      dpValve_nominal=6000,
      energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial)
      "Valve model, linear opening characteristics"
      annotation (Placement(transformation(extent={{154,-48},{174,-28}})));
    Modelica.Blocks.Sources.Ramp y1(
      height=1,
      duration=1,
      offset=0) "Control signal"
      annotation (Placement(transformation(extent={{96,78},{116,98}})));
    Modelica.Blocks.Sources.Ramp y2(
      height=1,
      duration=1,
      offset=0) "Control signal"
      annotation (Placement(transformation(extent={{-260,-258},{-240,-238}})));
    Buildings.DHC.Loads.BaseClasses.BuildingTimeSeries bui(
      filNam=
          "modelica://Buildings/Resources/Data/DHC/Loads/Examples/SwissResidential_20190916.mos",
      nPorts_bHeaWat=1,
      nPorts_aChiWat=2,
      nPorts_bChiWat=2,
      nPorts_aHeaWat=1)
      "Building"
      annotation (Placement(transformation(extent={{228,-76},{298,-4}})));

    Buildings.Fluid.Actuators.Valves.ThreeWayLinear valLin5(
      redeclare package Medium = Buildings.Media.Water,
      l={0.05,0.05},
      m_flow_nominal=2,
      use_inputFilter=false,
      dpValve_nominal=6000,
      energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial)
      "Valve model, linear opening characteristics"
      annotation (Placement(transformation(extent={{14,-124},{34,-104}})));
    Buildings.ThermalZones.Detailed.Validation.BaseClasses.SingleZoneFloor sinZonFlo(
      redeclare package Medium =
          Modelica.Media.Water.ConstantPropertyLiquidWater,
      use_windPressure=false,
      gai(K=0*[0.4; 0.4; 0.2]))
      "Single-zone floor model"
      annotation (Placement(transformation(extent={{250,-56},{274,-32}})));
    Generators.Digital_Twins.WolfCGB20_GC wolfCGB20_GC
      annotation (Placement(transformation(extent={{-458,-44},{-376,10}})));
    Generators.Digital_Twins.WolfCHA10_GC wolfCHA10_GC
      annotation (Placement(transformation(extent={{-428,-320},{-356,-248}})));
    Fluid.Pumps.Test_Pump_controler test_Pump_controler
      annotation (Placement(transformation(extent={{-200,-152},{-180,-132}})));
    Storage.StratifiedHeatStorage tan(
      redeclare package Medium = Buildings.Media.Water,
      m_flow_nominal=0.1,
      VTan=775,
      hTan=3,
      dIns=0.3,
      nSeg=10)
      annotation (Placement(transformation(extent={{-30,-42},{46,34}})));
  equation
    connect(bat.SOC,con. SOC) annotation (Line(
        points={{-176.5,-463.2},{-220,-463.2},{-220,-434},{-213.25,-434}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(con.y,bat. P) annotation (Line(
        points={{-191.375,-434},{-191.375,-436},{-149,-436},{-149,-454}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(bat.terminal,conv. terminal_p) annotation (Line(
        points={{-124,-477},{-44,-477},{-44,-451},{-30,-451}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(acLoad.terminal,gri. terminal) annotation (Line(
        points={{316,-372},{316,-410},{82,-410}},
        color={0,120,120},
        smooth=Smooth.None));
    connect(conv.terminal_n,gri. terminal) annotation (Line(
        points={{12,-451},{82,-451},{82,-410}},
        color={0,120,120},
        smooth=Smooth.None));
    connect(pv.weaBus, weaDat.weaBus) annotation (Line(
        points={{380,-365.5},{380,120},{230,120}},
        color={255,204,51},
        thickness=0.5));
    connect(theMixVal.yMixSet,sch. y[1]) annotation (Line(points={{245.8,-181.8},
            {242,-181.8},{242,-182},{240,-182},{240,-164},{436.8,-164},{436.8,
            -170}},             color={0,0,127}));
    connect(theMixVal.port_col, souCol.ports[1]) annotation (Line(points={{248,
            -212.2},{126,-212.2},{126,-184.7}},                   color={0,127,
            255}));
    connect(conTSetMix.y, theMixVal.TMixSet) annotation (Line(points={{438.9,
            -135},{232,-135},{232,-193.2},{245.8,-193.2}}, color={0,0,127}));
    connect(acLoad1.terminal, gri.terminal) annotation (Line(points={{250,-372},
            {250,-410},{82,-410}}, color={0,120,120}));
    connect(lin.terminal_p, pv.terminal) annotation (Line(points={{104,-489},{
            428,-489},{428,-397},{414,-397}}, color={0,0,255}));
    connect(lin.terminal_n, conv.terminal_p) annotation (Line(points={{54,-489},
            {-44,-489},{-44,-451},{-30,-451}},                       color={0,0,
            255}));
    connect(domHotWat.port_aDom, souCol.ports[2]) annotation (Line(points={{144,
            -246.4},{126,-246.4},{126,-181.3}}, color={0,127,255}));
    connect(domHotWat.port_bDom, theMixVal.port_hot) annotation (Line(points={{
            208,-246.4},{228,-246.4},{228,-204.6},{248,-204.6}}, color={0,127,
            255}));
    connect(conTSetHot.y, domHotWat.TDomSet) annotation (Line(points={{114.7,
            -259},{127.75,-259},{127.75,-268},{140.8,-268}}, color={0,0,127}));
    connect(domHotWat.PEle, acLoad1.Pow) annotation (Line(points={{211.2,-268},
            {211.2,-264},{250,-264},{250,-326}}, color={0,0,127}));
    connect(y.y, valLin2.y) annotation (Line(points={{-271,66},{-176,66},{-176,
            12}}, color={0,0,127}));
    connect(solCol.weaBus, weaDat.weaBus) annotation (Line(
        points={{-338,-107.08},{-360,-107.08},{-360,120},{230,120}},
        color={255,204,51},
        thickness=0.5));
    connect(y.y, valLin1.y) annotation (Line(points={{-271,66},{-176,66},{-176,
            64},{-120,64},{-120,-84}}, color={0,0,127}));
    connect(valLin6.y, y1.y)
      annotation (Line(points={{164,-26},{164,88},{117,88}}, color={0,0,127}));
    connect(y2.y, valLin3.y) annotation (Line(points={{-239,-248},{-200,-248},{
            -200,-262}}, color={0,0,127}));
    connect(valLin4.y, y2.y) annotation (Line(points={{-126,-298},{-128,-298},{
            -128,-248},{-239,-248}},             color={0,0,127}));
    connect(domHotWat.port_bHea, valLin6.port_3) annotation (Line(points={{144,
            -289.6},{84,-289.6},{84,-212},{80,-212},{80,-104},{164,-104},{164,
            -48}}, color={0,127,255}));
    connect(acLoad.Pow, bui.PPum) annotation (Line(points={{316,-328},{316,
            -32.8},{300.333,-32.8}},
                             color={0,0,127}));
    connect(bui.ports_bHeaWat[1], valLin5.port_1) annotation (Line(points={{298,
            -47.2},{304,-47.2},{304,-88},{8,-88},{8,-114},{14,-114}}, color={0,
            127,255}));
    connect(domHotWat.port_aHea, valLin5.port_3) annotation (Line(points={{208,
            -289.6},{220,-289.6},{220,-136},{24,-136},{24,-124}}, color={0,127,
            255}));
    connect(y1.y, valLin5.y) annotation (Line(points={{117,88},{164,88},{164,
            -16},{148,-16},{148,-92},{24,-92},{24,-102}}, color={0,0,127}));
    connect(solCol.port_a, valLin1.port_3) annotation (Line(points={{-338,-133},
            {-350,-133},{-350,-188},{-120,-188},{-120,-106}},
                    color={0,127,255}));
    connect(valLin4.port_3, bui.ports_aChiWat[1]) annotation (Line(points={{-126,
            -320},{-128,-320},{-128,-328},{88,-328},{88,-96},{216,-96},{216,
            -62.8},{228,-62.8}},     color={0,127,255}));
    connect(valLin3.port_3, bui.ports_bChiWat[1]) annotation (Line(points={{-200,
            -284},{-200,-288},{32,-288},{32,-132},{312,-132},{312,-62.8},{298,
            -62.8}},     color={0,127,255}));
    connect(bui.ports_aHeaWat[1], valLin6.port_2) annotation (Line(points={{228,
            -47.2},{188,-47.2},{188,-38},{174,-38}}, color={0,127,255}));
    connect(sinZonFlo.ports[1], bui.ports_aChiWat[2]) annotation (Line(points={{254.98,
            -51.2},{254.98,-60.4},{228,-60.4}},
                               color={0,127,255}));
    connect(sinZonFlo.ports[2], bui.ports_bChiWat[2]) annotation (Line(points={{255.58,
            -51.2},{256,-51.2},{256,-56},{296,-56},{296,-60.4},{298,-60.4}},
                        color={0,127,255}));
    connect(sinZonFlo.weaBus, weaDat.weaBus) annotation (Line(
        points={{254.08,-33.8},{254.08,120},{230,120}},
        color={255,204,51},
        thickness=0.5));
    connect(booleanExpression1.y, wolfCHA10_GC.HPOn) annotation (Line(points={{
            -467,-250},{-467,-252},{-440,-252},{-440,-264.92},{-424.04,-264.92}},
          color={255,0,255}));
    connect(booleanExpression.y, wolfCHA10_GC.HPMode) annotation (Line(points={
            {-465,-270},{-465,-272},{-440,-272},{-440,-278.6},{-424.04,-278.6}},
          color={255,0,255}));
    connect(realExpression2.y, wolfCHA10_GC.HPModulation) annotation (Line(
          points={{-467,-300},{-440,-300},{-440,-293},{-424.04,-293}}, color={0,
            0,127}));
    connect(realExpression5.y, wolfCHA10_GC.HPAuxModulation) annotation (Line(
          points={{-469,-320},{-444,-320},{-444,-306.68},{-424.04,-306.68}},
          color={0,0,127}));
    connect(wolfCHA10_GC.term_p, gri.terminal) annotation (Line(points={{
            -371.84,-332.24},{-371.84,-410},{82,-410}}, color={0,120,120}));
    connect(wolfCHA10_GC.port_a, valLin4.port_1) annotation (Line(points={{
            -342.32,-286.16},{-220,-286.16},{-220,-296},{-148,-296},{-148,-310},
            {-136,-310}}, color={0,127,255}));
    connect(wolfCHA10_GC.port_b, valLin3.port_1) annotation (Line(points={{
            -336.56,-270.32},{-273.28,-270.32},{-273.28,-274},{-210,-274}},
          color={0,127,255}));
    connect(wolfCHA10_GC.weaBus, weaDat.weaBus) annotation (Line(
        points={{-371.12,-242.24},{-371.12,-60},{-360,-60},{-360,120},{230,120}},

        color={255,204,51},
        thickness=0.5));
    connect(realExpression.y, wolfCGB20_GC.ControlIn) annotation (Line(points={
            {-493,-14},{-493,-16.19},{-461.007,-16.19}}, color={0,0,127}));
    connect(wolfCGB20_GC.term_p, gri.terminal) annotation (Line(points={{
            -414.267,-46.7},{-414.267,-64},{-364,-64},{-364,-406},{-366,-406},{
            -366,-410},{82,-410}}, color={0,120,120}));
    connect(wolfCGB20_GC.port_b, valLin1.port_3) annotation (Line(points={{
            -379.28,-6.74},{-200,-6.74},{-200,-96},{-140,-96},{-140,-108},{-120,
            -108},{-120,-106}}, color={0,127,255}));
    connect(wolfCGB20_GC.port_a, valLin2.port_1) annotation (Line(points={{
            -365.067,-22.94},{-196,-22.94},{-196,0},{-186,0}}, color={0,127,255}));
    connect(wolfCGB20_GC.weaBus, weaDat.weaBus) annotation (Line(
        points={{-427.387,15.94},{-427.387,120},{230,120}},
        color={255,204,51},
        thickness=0.5));
    connect(test_Pump_controler.port_a, solCol.port_b) annotation (Line(points=
            {{-199.6,-141.8},{-268,-141.8},{-268,-133},{-278,-133}}, color={0,
            127,255}));
    connect(test_Pump_controler.port_b, valLin2.port_3) annotation (Line(points
          ={{-179.6,-141.8},{-176,-141.8},{-176,-10}}, color={0,127,255}));
    connect(realExpression1.y, test_Pump_controler.SetInSignal) annotation (
        Line(points={{-189,-118},{-196,-118},{-196,-131},{-198.2,-131}}, color=
            {0,0,127}));
    connect(test_Pump_controler.term_p, gri.terminal) annotation (Line(points={
            {-184.4,-155.4},{-184.4,-154},{-164,-154},{-164,-410},{82,-410}},
          color={0,120,120}));
    connect(tan.port_a, valLin2.port_2) annotation (Line(points={{8,34},{8,64},
            {-96,64},{-96,0},{-166,0}}, color={0,127,255}));
    connect(tan.port_a, valLin4.port_2) annotation (Line(points={{8,34},{8,40},
            {-90,40},{-90,-310},{-116,-310}}, color={0,127,255}));
    connect(tan.port_b, valLin6.port_1) annotation (Line(points={{8,-42},{8,-56},
            {144,-56},{144,-38},{154,-38}}, color={0,127,255}));
    connect(tan.fluPorVol[2], valLin1.port_2) annotation (Line(points={{-11,
            -9.32},{-11,-14},{-64,-14},{-64,-96},{-110,-96}}, color={0,127,255}));
    connect(tan.fluPorVol[3], valLin3.port_2) annotation (Line(points={{-11,
            -7.8},{-11,-10},{-78,-10},{-78,-274},{-190,-274}}, color={0,127,255}));
    connect(tan.fluPorVol[4], valLin5.port_2) annotation (Line(points={{-11,
            -6.28},{-11,-6},{-68,-6},{-68,-142},{50,-142},{50,-114},{34,-114}},
          color={0,127,255}));
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
        StartTime=12000,
        StopTime=90000,
        Tolerance=1e-06,
        __Dymola_Algorithm="Dassl"),
      Documentation(info="<html>
<p>
This model provides an example for a building with loads provided
as time series and connected to a direct ETS for heating with the
return heating water temperature controlled below a maximum threshold.
</p>
</html>",   revisions="<html>
<ul>
<li>
March 20, 2022, by Chengnan Shi:<br/>
First implementation.
</li>
</ul>
</html>"));
  end SF2;

  model SF3 "Example model of a building with loads provided as time series"
    extends Modelica.Icons.Example;
    package Medium=Buildings.Media.Water
      "Medium model";
    Buildings.Electrical.DC.Sources.PVSimpleOriented
                                           pv(
      A=200e3/800/0.12,
      til=0.34906585039887,
      azi=-0.78539816339745,
      V_nominal=480) "PV array"
      annotation (Placement(transformation(extent={{414,-432},{346,-362}})));
    Buildings.Electrical.DC.Storage.Battery bat(EMax=500e3*4*3600, V_nominal=
          480)       "Battery"
      annotation (Placement(transformation(extent={{-124,-500},{-174,-454}})));
    Buildings.Electrical.AC.OnePhase.Conversion.ACDCConverter conv(
        conversionFactor=480/480, eta=0.9)
               "AC/DC converter"
      annotation (Placement(transformation(extent={{12,-474},{-30,-428}})));
    Buildings.Electrical.AC.OnePhase.Sources.Grid gri(
      f=60,
      V=480,
      phiSou=0) annotation (Placement(transformation(extent={{62,-410},{102,
              -370}})));
    Buildings.Examples.ChillerPlant.BaseClasses.Controls.BatteryControl
                                        con "Battery controller"
      annotation (Placement(transformation(extent={{-212,-444},{-192,-424}})));
    Buildings.Electrical.AC.OnePhase.Loads.Inductive acLoad(mode=Buildings.Electrical.Types.Load.VariableZ_P_input,
        V_nominal=480)
      annotation (Placement(transformation(extent={{22,-20},{-22,20}},
          rotation=270,
          origin={300,-350})));
    Buildings.BoundaryConditions.WeatherData.ReaderTMY3 weaDat(filNam=
          ModelicaServices.ExternalReferences.loadResource(
          "modelica://Buildings/Resources/weatherdata/USA_CA_San.Francisco.Intl.AP.724940_TMY3.mos"))
      annotation (Placement(transformation(extent={{210,110},{230,130}})));
    Modelica.Blocks.Sources.RealExpression realExpression(y=1)
      annotation (Placement(transformation(extent={{-532,-50},{-512,-30}})));
    Buildings.Fluid.Actuators.Valves.ThreeWayLinear valLin1(
      redeclare package Medium = Buildings.Media.Water,
      l={0.05,0.05},
      m_flow_nominal=2,
      use_inputFilter=false,
      dpValve_nominal=6000,
      energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial)
      "Valve model, linear opening characteristics"
      annotation (Placement(transformation(extent={{-220,-68},{-200,-48}})));
    Modelica.Blocks.Sources.Ramp y(
      height=1,
      duration=1,
      offset=0) "Control signal"
      annotation (Placement(transformation(extent={{-292,56},{-272,76}})));

    parameter Buildings.DHC.Loads.HotWater.Data.GenericDomesticHotWaterWithHeatExchanger
      datWatHea(VTan=0.1892706, mDom_flow_nominal=6.52944E-06*1000)
      "Data for heat pump water heater with tank"
      annotation (Placement(transformation(extent={{438,-224},{458,-204}})));
    Modelica.Blocks.Sources.Constant conTSetHot(k(
        final unit="K",
        displayUnit="degC") = 313.15)
      "Temperature setpoint for hot water supply to fixture"
      annotation (Placement(transformation(extent={{100,-266},{114,-252}})));
    Buildings.Fluid.Sources.Boundary_pT souCol(
      nPorts=2,
      redeclare package Medium = Medium,
      T(displayUnit="degC"))          "Source of domestic cold water"
      annotation (Placement(transformation(extent={{-17,-17},{17,17}},
          rotation=0,
          origin={107,-207})));
    Modelica.Blocks.Sources.CombiTimeTable sch(
      tableOnFile=true,
      tableName="tab1",
      fileName=Modelica.Utilities.Files.loadResource(
          "modelica://Buildings/Resources/Data/DHC/Loads/HotWater/DHW_ApartmentMidRise.mos"),
      smoothness=Modelica.Blocks.Types.Smoothness.ContinuousDerivative,
      extrapolation=Modelica.Blocks.Types.Extrapolation.Periodic)
      "Domestic hot water fixture draw fraction schedule"
      annotation (Placement(transformation(extent={{-12,-12},{12,12}},
          rotation=180,
          origin={450,-170})));

    Buildings.DHC.Loads.HotWater.ThermostaticMixingValve theMixVal(redeclare
        package Medium = Medium, mMix_flow_nominal=1.2*datWatHea.mDom_flow_nominal)
      annotation (Placement(transformation(extent={{248,-216},{292,-178}})));
    Modelica.Blocks.Sources.Constant conTSetMix(k(
        final unit="K",
        displayUnit="degC") = 308.15)
      "Temperature setpoint for mixed water supply to fixture"
      annotation (Placement(transformation(extent={{-11,-11},{11,11}},
          rotation=180,
          origin={451,-135})));
    Buildings.Electrical.AC.OnePhase.Loads.Inductive acLoad1(mode=Buildings.Electrical.Types.Load.VariableZ_P_input,
        V_nominal=480)
      annotation (Placement(transformation(extent={{23,-22},{-23,22}},
          rotation=270,
          origin={250,-349})));
    Buildings.Electrical.DC.Lines.TwoPortResistance lin(R=0.05)
      "Transmission line"
      annotation (Placement(transformation(extent={{-25,-17},{25,17}},
          rotation=0,
          origin={85,-489})));
    Modelica.Blocks.Sources.RealExpression realExpression1(y=10)
      annotation (Placement(transformation(extent={{-216,-292},{-196,-272}})));
    CoSES_ProHMo.Consumer.MyDHW domHotWat(
      redeclare package MediumDom = Buildings.Media.Water,
      redeclare package MediumHea = Buildings.Media.Water,
      dat=datWatHea)
      annotation (Placement(transformation(extent={{144,-304},{208,-232}})));
    Buildings.Fluid.Actuators.Valves.ThreeWayLinear valLin2(
      redeclare package Medium = Buildings.Media.Water,
      l={0.05,0.05},
      m_flow_nominal=2,
      use_inputFilter=false,
      dpValve_nominal=6000,
      energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial)
      "Valve model, linear opening characteristics"
      annotation (Placement(transformation(extent={{-152,-160},{-132,-140}})));
    Modelica.Blocks.Sources.BooleanStep uHea(startTime(displayUnit="min") =
        60000)
      "On/off signal"
      annotation (Placement(transformation(extent={{-528,-14},{-496,18}})));
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
      annotation (Placement(transformation(extent={{-322,-324},{-262,-270}})));

    Buildings.DHC.Loads.BaseClasses.BuildingTimeSeries buiHea(
      have_chiWat=false,
      filNam=
          "modelica://Buildings/Resources/Data/DHC/Loads/Examples/SwissResidential_20190916.mos",
      nPorts_aChiWat=1,
      nPorts_bChiWat=1,
      nPorts_aHeaWat=2,
      nPorts_bHeaWat=2)
      "Building with heating only"
      annotation (Placement(transformation(extent={{184,-38},{258,32}})));

    Buildings.ThermalZones.Detailed.Validation.BaseClasses.SingleZoneFloor sinZonFlo(
      redeclare package Medium =
          Modelica.Media.Water.ConstantPropertyLiquidWater,
      use_windPressure=false,
      gai(K=0*[0.4; 0.4; 0.2]))
      "Single-zone floor model"
      annotation (Placement(transformation(extent={{212,-14},{232,6}})));
    Generators.Digital_Twins.WolfBWS1_10_GC wolfBWS1_10_GC
      annotation (Placement(transformation(extent={{-466,-58},{-382,26}})));
    Fluid.Pumps.Test_Pump_controler test_Pump_controler
      annotation (Placement(transformation(extent={{-178,-306},{-158,-286}})));
    Storage.StratifiedHeatStorage tan(
      redeclare package Medium = Buildings.Media.Water,
      m_flow_nominal=0.1,
      VTan=775,
      hTan=3,
      dIns=0.3,
      nSeg=10)
      annotation (Placement(transformation(extent={{-42,-34},{34,42}})));
  equation
    connect(bat.SOC,con. SOC) annotation (Line(
        points={{-176.5,-463.2},{-220,-463.2},{-220,-434},{-213.25,-434}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(con.y,bat. P) annotation (Line(
        points={{-191.375,-434},{-191.375,-436},{-149,-436},{-149,-454}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(bat.terminal,conv. terminal_p) annotation (Line(
        points={{-124,-477},{-44,-477},{-44,-451},{-30,-451}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(acLoad.terminal,gri. terminal) annotation (Line(
        points={{300,-372},{300,-410},{82,-410}},
        color={0,120,120},
        smooth=Smooth.None));
    connect(conv.terminal_n,gri. terminal) annotation (Line(
        points={{12,-451},{82,-451},{82,-410}},
        color={0,120,120},
        smooth=Smooth.None));
    connect(pv.weaBus, weaDat.weaBus) annotation (Line(
        points={{380,-365.5},{380,120},{230,120}},
        color={255,204,51},
        thickness=0.5));
    connect(theMixVal.yMixSet,sch. y[1]) annotation (Line(points={{245.8,-181.8},
            {242,-181.8},{242,-182},{240,-182},{240,-164},{436.8,-164},{436.8,
            -170}},             color={0,0,127}));
    connect(theMixVal.port_col, souCol.ports[1]) annotation (Line(points={{248,
            -212.2},{248,-208.7},{124,-208.7}},                   color={0,127,
            255}));
    connect(conTSetMix.y, theMixVal.TMixSet) annotation (Line(points={{438.9,
            -135},{232,-135},{232,-193.2},{245.8,-193.2}}, color={0,0,127}));
    connect(acLoad1.terminal, gri.terminal) annotation (Line(points={{250,-372},
            {250,-410},{82,-410}}, color={0,120,120}));
    connect(lin.terminal_p, pv.terminal) annotation (Line(points={{110,-489},{
            428,-489},{428,-397},{414,-397}}, color={0,0,255}));
    connect(lin.terminal_n, conv.terminal_p) annotation (Line(points={{60,-489},
            {24,-489},{24,-488},{-44,-488},{-44,-451},{-30,-451}},   color={0,0,
            255}));
    connect(domHotWat.port_aDom, souCol.ports[2]) annotation (Line(points={{144,
            -246.4},{140,-246.4},{140,-205.3},{124,-205.3}},
                                                color={0,127,255}));
    connect(domHotWat.port_bDom, theMixVal.port_hot) annotation (Line(points={{
            208,-246.4},{228,-246.4},{228,-204.6},{248,-204.6}}, color={0,127,
            255}));
    connect(conTSetHot.y, domHotWat.TDomSet) annotation (Line(points={{114.7,
            -259},{127.75,-259},{127.75,-268},{140.8,-268}}, color={0,0,127}));
    connect(domHotWat.PEle, acLoad1.Pow) annotation (Line(points={{211.2,-268},
            {211.2,-264},{250,-264},{250,-326}}, color={0,0,127}));
    connect(y.y, valLin2.y) annotation (Line(points={{-271,66},{-188,66},{-188,
            -138},{-142,-138}},
                  color={0,0,127}));
    connect(solCol.weaBus, weaDat.weaBus) annotation (Line(
        points={{-322,-271.08},{-322,120},{230,120}},
        color={255,204,51},
        thickness=0.5));
    connect(solCol.port_a, valLin1.port_1) annotation (Line(points={{-322,-297},
            {-336,-297},{-336,-58},{-220,-58}},                         color={
            0,127,255}));
    connect(y.y, valLin1.y) annotation (Line(points={{-271,66},{-210,66},{-210,
            -46}},                     color={0,0,127}));
    connect(acLoad.Pow, buiHea.PPum) annotation (Line(points={{300,-328},{300,
            -228},{304,-228},{304,4},{260.467,4}}, color={0,0,127}));
    connect(sinZonFlo.weaBus, weaDat.weaBus) annotation (Line(
        points={{215.4,4.5},{215.4,104},{240,104},{240,120},{230,120}},
        color={255,204,51},
        thickness=0.5));
    connect(wolfBWS1_10_GC.HPOn, uHea.y) annotation (Line(points={{-464.74,2.06},
            {-479.57,2.06},{-479.57,2},{-494.4,2}}, color={255,0,255}));
    connect(realExpression.y, wolfBWS1_10_GC.HPAuxModulation) annotation (Line(
          points={{-511,-40},{-487.87,-40},{-487.87,-39.94},{-464.74,-39.94}},
          color={0,0,127}));
    connect(wolfBWS1_10_GC.weaBus, weaDat.weaBus) annotation (Line(
        points={{-418.96,35.24},{-418.96,120},{230,120}},
        color={255,204,51},
        thickness=0.5));
    connect(wolfBWS1_10_GC.port_b, valLin1.port_3) annotation (Line(points={{
            -340,10.88},{-356,10.88},{-356,-68},{-210,-68}}, color={0,127,255}));
    connect(wolfBWS1_10_GC.port_a, valLin2.port_3) annotation (Line(points={{
            -338.32,-15.16},{-338.32,-160},{-142,-160}}, color={0,127,255}));
    connect(wolfBWS1_10_GC.term_p, gri.terminal) annotation (Line(points={{
            -400.48,-72.28},{-400.48,-410},{82,-410}}, color={0,120,120}));
    connect(test_Pump_controler.port_a, solCol.port_b) annotation (Line(points=
            {{-177.6,-295.8},{-219.8,-295.8},{-219.8,-297},{-262,-297}}, color=
            {0,127,255}));
    connect(test_Pump_controler.port_b, valLin2.port_3) annotation (Line(points
          ={{-157.6,-295.8},{-142,-295.8},{-142,-160}}, color={0,127,255}));
    connect(test_Pump_controler.SetInSignal, realExpression1.y) annotation (
        Line(points={{-176.2,-285},{-176.2,-282},{-195,-282}}, color={0,0,127}));
    connect(test_Pump_controler.term_p, gri.terminal) annotation (Line(points={
            {-162.4,-309.4},{-162.4,-410},{82,-410}}, color={0,120,120}));
    connect(tan.port_a, valLin2.port_2) annotation (Line(points={{-4,42},{-4,56},
            {-60,56},{-60,-150},{-132,-150}}, color={0,127,255}));
    connect(tan.port_b, buiHea.ports_aHeaWat[1]) annotation (Line(points={{-4,
            -34},{-4,-48},{172,-48},{172,-11.1667},{184,-11.1667}}, color={0,
            127,255}));
    connect(sinZonFlo.ports[1], buiHea.ports_aHeaWat[2]) annotation (Line(
          points={{216.15,-10},{200,-10},{200,-8.83333},{184,-8.83333}}, color=
            {0,127,255}));
    connect(tan.fluPorVol[4], buiHea.ports_bHeaWat[1]) annotation (Line(points=
            {{-23,1.72},{-24,1.72},{-24,2},{-32,2},{-32,-66},{276,-66},{276,
            -11.1667},{258,-11.1667}}, color={0,127,255}));
    connect(sinZonFlo.ports[2], buiHea.ports_bHeaWat[2]) annotation (Line(
          points={{216.65,-10},{238,-10},{238,-8.83333},{258,-8.83333}}, color=
            {0,127,255}));
    connect(tan.fluPorVol[2], valLin1.port_2) annotation (Line(points={{-23,
            -1.32},{-23,2},{-24,2},{-24,-2},{-112,-2},{-112,-58},{-200,-58}},
          color={0,127,255}));
    connect(tan.fluPorVol[6], domHotWat.port_aHea) annotation (Line(points={{
            -23,4.76},{-23,10},{-44,10},{-44,-72},{212,-72},{212,-264},{220,
            -264},{220,-289.6},{208,-289.6}}, color={0,127,255}));
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
        StartTime=12000,
        StopTime=90000,
        Tolerance=1e-06,
        __Dymola_Algorithm="Dassl"),
      Documentation(info="<html>
<p>
This model provides an example for a building with loads provided
as time series and connected to a direct ETS for heating with the
return heating water temperature controlled below a maximum threshold.
</p>
</html>",   revisions="<html>
<ul>
<li>
March 20, 2022, by Chengnan Shi:<br/>
First implementation.
</li>
</ul>
</html>"));
  end SF3;

  model SF4 "Example model of a building with loads provided as time series and
  connected to an ETS for cooling"
    extends Modelica.Icons.Example;
    package Medium=Buildings.Media.Water
      "Medium model";
    Modelica.Blocks.Sources.Constant TDisRetSet(k=273.15 + 45)
      "Setpoint for district return temperature"
      annotation (Placement(transformation(extent={{-10,-10},{10,10}},
          rotation=180,
          origin={436,22})));
    Buildings.Electrical.DC.Sources.PVSimpleOriented
                                           pv(
      A=200e3/800/0.12,
      til=0.34906585039887,
      azi=-0.78539816339745,
      V_nominal=480) "PV array"
      annotation (Placement(transformation(extent={{414,-432},{346,-362}})));
    Buildings.Electrical.DC.Storage.Battery bat(EMax=500e3*4*3600, V_nominal=
          480)       "Battery"
      annotation (Placement(transformation(extent={{-124,-500},{-174,-454}})));
    Buildings.Electrical.AC.OnePhase.Conversion.ACDCConverter conv(
        conversionFactor=480/480, eta=0.9)
               "AC/DC converter"
      annotation (Placement(transformation(extent={{12,-474},{-30,-428}})));
    Buildings.Electrical.AC.OnePhase.Sources.Grid gri(
      f=60,
      V=480,
      phiSou=0) annotation (Placement(transformation(extent={{62,-410},{102,
              -370}})));
    Buildings.Examples.ChillerPlant.BaseClasses.Controls.BatteryControl
                                        con "Battery controller"
      annotation (Placement(transformation(extent={{-212,-444},{-192,-424}})));
    Buildings.Electrical.AC.OnePhase.Loads.Inductive acLoad(mode=Buildings.Electrical.Types.Load.VariableZ_P_input,
        V_nominal=480)
      annotation (Placement(transformation(extent={{22,-20},{-22,20}},
          rotation=270,
          origin={318,-350})));
    Buildings.BoundaryConditions.WeatherData.ReaderTMY3 weaDat(filNam=
          ModelicaServices.ExternalReferences.loadResource(
          "modelica://Buildings/Resources/weatherdata/USA_CA_San.Francisco.Intl.AP.724940_TMY3.mos"))
      annotation (Placement(transformation(extent={{210,110},{230,130}})));
    Modelica.Blocks.Sources.RealExpression realExpression(y=1)
      annotation (Placement(transformation(extent={{-532,6},{-512,26}})));

    parameter Buildings.DHC.Loads.HotWater.Data.GenericDomesticHotWaterWithHeatExchanger
      datWatHea(VTan=0.1892706, mDom_flow_nominal=6.52944E-06*1000)
      "Data for heat pump water heater with tank"
      annotation (Placement(transformation(extent={{438,-224},{458,-204}})));
    Modelica.Blocks.Sources.Constant conTSetHot(k(
        final unit="K",
        displayUnit="degC") = 313.15)
      "Temperature setpoint for hot water supply to fixture"
      annotation (Placement(transformation(extent={{88,-274},{102,-260}})));
    Buildings.Fluid.Sources.Boundary_pT souCol(
      nPorts=2,
      redeclare package Medium = Medium,
      T(displayUnit="degC"))          "Source of domestic cold water"
      annotation (Placement(transformation(extent={{-17,-17},{17,17}},
          rotation=0,
          origin={109,-183})));
    Modelica.Blocks.Sources.CombiTimeTable sch(
      tableOnFile=true,
      tableName="tab1",
      fileName=Modelica.Utilities.Files.loadResource(
          "modelica://Buildings/Resources/Data/DHC/Loads/HotWater/DHW_ApartmentMidRise.mos"),
      smoothness=Modelica.Blocks.Types.Smoothness.ContinuousDerivative,
      extrapolation=Modelica.Blocks.Types.Extrapolation.Periodic)
      "Domestic hot water fixture draw fraction schedule"
      annotation (Placement(transformation(extent={{-12,-12},{12,12}},
          rotation=180,
          origin={450,-170})));

    Buildings.DHC.Loads.HotWater.ThermostaticMixingValve theMixVal(redeclare
        package Medium = Medium, mMix_flow_nominal=1.2*datWatHea.mDom_flow_nominal)
      annotation (Placement(transformation(extent={{248,-216},{292,-178}})));
    Modelica.Blocks.Sources.Constant conTSetMix(k(
        final unit="K",
        displayUnit="degC") = 308.15)
      "Temperature setpoint for mixed water supply to fixture"
      annotation (Placement(transformation(extent={{-11,-11},{11,11}},
          rotation=180,
          origin={451,-135})));
    Buildings.Electrical.AC.OnePhase.Loads.Inductive acLoad1(mode=Buildings.Electrical.Types.Load.VariableZ_P_input,
        V_nominal=480)
      annotation (Placement(transformation(extent={{23,-22},{-23,22}},
          rotation=270,
          origin={250,-349})));
    Buildings.Electrical.DC.Lines.TwoPortResistance lin(R=0.05)
      "Transmission line"
      annotation (Placement(transformation(extent={{-25,-17},{25,17}},
          rotation=0,
          origin={73,-489})));
    CoSES_ProHMo.Consumer.MyDHW domHotWat(
      redeclare package MediumDom = Buildings.Media.Water,
      redeclare package MediumHea = Buildings.Media.Water,
      dat=datWatHea)
      annotation (Placement(transformation(extent={{144,-304},{208,-232}})));
    Buildings.Applications.DataCenters.ChillerCooled.Equipment.ElectricHeater eleHea(
      redeclare package Medium = Buildings.Media.Water,
      m_flow_nominal=10,
      QMax_flow=1000,
      energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
      dp_nominal=1000,
      eta=0.95)
      "Electric heater"
      annotation (Placement(transformation(extent={{-264,-226},{-216,-172}})));
    Modelica.Blocks.Sources.BooleanStep uHea(startTime(displayUnit="min") =
        60000)
      "On/off signal"
      annotation (Placement(transformation(extent={{-524,-224},{-504,-204}})));
    Modelica.Blocks.Sources.Constant conTSetHot1(k(
        final unit="K",
        displayUnit="degC") = 293.15)
      "Temperature setpoint for hot water supply to fixture"
      annotation (Placement(transformation(extent={{-524,-188},{-506,-170}})));
    Buildings.Electrical.AC.OnePhase.Loads.Inductive acLoad2(mode=Buildings.Electrical.Types.Load.VariableZ_P_input,
        V_nominal=480)
      annotation (Placement(transformation(extent={{23,-22},{-23,22}},
          rotation=270,
          origin={-210,-357})));
    Buildings.DHC.Loads.BaseClasses.BuildingTimeSeries buiHea(
      have_chiWat=false,
      filNam=
          "modelica://Buildings/Resources/Data/DHC/Loads/Examples/SwissResidential_20190916.mos",
      nPorts_aChiWat=1,
      nPorts_bChiWat=1,
      nPorts_aHeaWat=2,
      nPorts_bHeaWat=2)
      "Building with heating only"
      annotation (Placement(transformation(extent={{210,-82},{282,-10}})));

    Buildings.ThermalZones.Detailed.Validation.BaseClasses.SingleZoneFloor sinZonFlo(
      redeclare package Medium =
          Modelica.Media.Water.ConstantPropertyLiquidWater,
      use_windPressure=false,
      gai(K=0*[0.4; 0.4; 0.2]))
      "Single-zone floor model"
      annotation (Placement(transformation(extent={{234,-62},{256,-40}})));
    Generators.Digital_Twins.WolfCGB20_GC wolfCGB20_GC
      annotation (Placement(transformation(extent={{-436,-34},{-346,26}})));
    Storage.StratifiedHeatStorage tan1(
      redeclare package Medium = Buildings.Media.Water,
      m_flow_nominal=0.1,
      VTan=775,
      hTan=3,
      dIns=0.3,
      nSeg=10) annotation (Placement(transformation(extent={{-78,-70},{-2,6}})));
  equation
    connect(bat.SOC,con. SOC) annotation (Line(
        points={{-176.5,-463.2},{-220,-463.2},{-220,-434},{-213.25,-434}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(con.y,bat. P) annotation (Line(
        points={{-191.375,-434},{-191.375,-436},{-149,-436},{-149,-454}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(bat.terminal,conv. terminal_p) annotation (Line(
        points={{-124,-477},{-44,-477},{-44,-451},{-30,-451}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(acLoad.terminal,gri. terminal) annotation (Line(
        points={{318,-372},{318,-410},{82,-410}},
        color={0,120,120},
        smooth=Smooth.None));
    connect(conv.terminal_n,gri. terminal) annotation (Line(
        points={{12,-451},{82,-451},{82,-410}},
        color={0,120,120},
        smooth=Smooth.None));
    connect(pv.weaBus, weaDat.weaBus) annotation (Line(
        points={{380,-365.5},{380,120},{230,120}},
        color={255,204,51},
        thickness=0.5));
    connect(theMixVal.yMixSet,sch. y[1]) annotation (Line(points={{245.8,-181.8},
            {242,-181.8},{242,-182},{240,-182},{240,-164},{436.8,-164},{436.8,
            -170}},             color={0,0,127}));
    connect(theMixVal.port_col, souCol.ports[1]) annotation (Line(points={{248,
            -212.2},{126,-212.2},{126,-184.7}},                   color={0,127,
            255}));
    connect(conTSetMix.y, theMixVal.TMixSet) annotation (Line(points={{438.9,
            -135},{232,-135},{232,-193.2},{245.8,-193.2}}, color={0,0,127}));
    connect(acLoad1.terminal, gri.terminal) annotation (Line(points={{250,-372},
            {250,-410},{82,-410}}, color={0,120,120}));
    connect(lin.terminal_p, pv.terminal) annotation (Line(points={{98,-489},{
            428,-489},{428,-397},{414,-397}}, color={0,0,255}));
    connect(lin.terminal_n, conv.terminal_p) annotation (Line(points={{48,-489},
            {-44,-489},{-44,-451},{-30,-451}},                       color={0,0,
            255}));
    connect(domHotWat.port_aDom, souCol.ports[2]) annotation (Line(points={{144,
            -246.4},{126,-246.4},{126,-181.3}}, color={0,127,255}));
    connect(domHotWat.port_bDom, theMixVal.port_hot) annotation (Line(points={{
            208,-246.4},{228,-246.4},{228,-204.6},{248,-204.6}}, color={0,127,
            255}));
    connect(conTSetHot.y, domHotWat.TDomSet) annotation (Line(points={{102.7,
            -267},{121.75,-267},{121.75,-268},{140.8,-268}}, color={0,0,127}));
    connect(domHotWat.PEle, acLoad1.Pow) annotation (Line(points={{211.2,-268},
            {211.2,-264},{250,-264},{250,-326}}, color={0,0,127}));
    connect(uHea.y, eleHea.on) annotation (Line(points={{-503,-214},{-284,-214},
            {-284,-190.9},{-268.8,-190.9}}, color={255,0,255}));
    connect(conTSetHot1.y, eleHea.TSet) annotation (Line(points={{-505.1,-179},
            {-500,-179},{-500,-177.4},{-268.8,-177.4}}, color={0,0,127}));
    connect(eleHea.P, acLoad2.Pow) annotation (Line(points={{-213.6,-215.2},{
            -204,-215.2},{-204,-320},{-210,-320},{-210,-334}}, color={0,0,127}));
    connect(acLoad2.terminal, gri.terminal) annotation (Line(points={{-210,-380},
            {-212,-380},{-212,-410},{82,-410}}, color={0,120,120}));
    connect(buiHea.PPum, acLoad.Pow) annotation (Line(points={{284.4,-38.8},{
            304,-38.8},{304,-312},{318,-312},{318,-328}},
                                                    color={0,0,127}));
    connect(sinZonFlo.ports[2], buiHea.ports_bHeaWat[1]) annotation (Line(
          points={{239.115,-57.6},{263.42,-57.6},{263.42,-54.4},{282,-54.4}},
                        color={0,127,255}));
    connect(sinZonFlo.ports[1], buiHea.ports_aHeaWat[1]) annotation (Line(
          points={{238.565,-57.6},{220,-57.6},{220,-54.4},{210,-54.4}},
                        color={0,127,255}));
    connect(sinZonFlo.weaBus, weaDat.weaBus) annotation (Line(
        points={{237.74,-41.65},{237.74,-40},{238,-40},{238,104},{240,104},{240,
            120},{230,120}},
        color={255,204,51},
        thickness=0.5));
    connect(realExpression.y, wolfCGB20_GC.ControlIn) annotation (Line(points={
            {-511,16},{-504,16},{-504,-3.1},{-439.3,-3.1}}, color={0,0,127}));
    connect(wolfCGB20_GC.weaBus, weaDat.weaBus) annotation (Line(
        points={{-402.4,32.6},{-402.4,120},{230,120}},
        color={255,204,51},
        thickness=0.5));
    connect(tan1.port_a, wolfCGB20_GC.port_a) annotation (Line(points={{-40,6},
            {-40,28},{-334,28},{-334,-10.6}}, color={0,127,255}));
    connect(tan1.port_b, buiHea.ports_aHeaWat[2]) annotation (Line(points={{-40,
            -70},{-40,-64},{-44,-64},{-44,-72},{200,-72},{200,-52},{210,-52}},
          color={0,127,255}));
    connect(domHotWat.port_bHea, tan1.port_b) annotation (Line(points={{144,
            -289.6},{80,-289.6},{80,-72},{-40,-72},{-40,-70}}, color={0,127,255}));
    connect(buiHea.ports_bHeaWat[2], tan1.fluPorVol[4]) annotation (Line(points
          ={{282,-52},{296,-52},{296,-104},{-92,-104},{-92,-34.28},{-59,-34.28}},
          color={0,127,255}));
    connect(domHotWat.port_aHea, tan1.fluPorVol[6]) annotation (Line(points={{
            208,-289.6},{220,-289.6},{220,-316},{-100,-316},{-100,-31.24},{-59,
            -31.24}}, color={0,127,255}));
    connect(tan1.fluPorVol[2], wolfCGB20_GC.port_b) annotation (Line(points={{
            -59,-37.32},{-60,-37.32},{-60,-46},{-242,-46},{-242,7.4},{-349.6,
            7.4}}, color={0,127,255}));
    connect(tan1.fluPorVol[8], eleHea.port_b) annotation (Line(points={{-59,
            -28.2},{-59,-24},{-92,-24},{-92,-199},{-216,-199}}, color={0,127,
            255}));
    connect(eleHea.port_a, tan1.fluPorVol[3]) annotation (Line(points={{-264,
            -199},{-288,-199},{-288,-42},{-58,-42},{-58,-35.8},{-59,-35.8}},
          color={0,127,255}));
    connect(wolfCGB20_GC.term_p, gri.terminal) annotation (Line(points={{-388,
            -37},{-388,-410},{82,-410}}, color={0,120,120}));
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
        StartTime=12000,
        StopTime=90000,
        Tolerance=1e-06,
        __Dymola_Algorithm="Dassl"),
      Documentation(info="<html>
<p>
This model provides an example for a building with loads provided
as time series and connected to a direct ETS for heating with the
return heating water temperature controlled below a maximum threshold.
</p>
</html>",   revisions="<html>
<ul>
<li>
March 20, 2022, by Chengnan Shi:<br/>
First implementation.
</li>
</ul>
</html>"));
  end SF4;

  model MF5 "Example model of a building with loads provided as time series"
    extends Modelica.Icons.Example;
    package Medium=Buildings.Media.Water
      "Medium model";
    Buildings.Electrical.DC.Sources.PVSimpleOriented
                                           pv(
      A=200e3/800/0.12,
      til=0.34906585039887,
      azi=-0.78539816339745,
      V_nominal=480) "PV array"
      annotation (Placement(transformation(extent={{414,-432},{346,-362}})));
    Buildings.Electrical.DC.Storage.Battery bat(EMax=500e3*4*3600, V_nominal=
          480)       "Battery"
      annotation (Placement(transformation(extent={{-124,-500},{-174,-454}})));
    Buildings.Electrical.AC.OnePhase.Conversion.ACDCConverter conv(
        conversionFactor=480/480, eta=0.9)
               "AC/DC converter"
      annotation (Placement(transformation(extent={{12,-474},{-30,-428}})));
    Buildings.Electrical.AC.OnePhase.Sources.Grid gri(
      f=60,
      V=480,
      phiSou=0) annotation (Placement(transformation(extent={{62,-410},{102,
              -370}})));
    Buildings.Examples.ChillerPlant.BaseClasses.Controls.BatteryControl
                                        con "Battery controller"
      annotation (Placement(transformation(extent={{-212,-444},{-192,-424}})));
    Buildings.Electrical.AC.OnePhase.Loads.Inductive acLoad(mode=Buildings.Electrical.Types.Load.VariableZ_P_input,
        V_nominal=480)
      annotation (Placement(transformation(extent={{22,-20},{-22,20}},
          rotation=270,
          origin={320,-350})));
    Buildings.BoundaryConditions.WeatherData.ReaderTMY3 weaDat(filNam=
          ModelicaServices.ExternalReferences.loadResource(
          "modelica://Buildings/Resources/weatherdata/USA_CA_San.Francisco.Intl.AP.724940_TMY3.mos"))
      annotation (Placement(transformation(extent={{210,110},{230,130}})));
    Modelica.Blocks.Sources.RealExpression realExpression(y=1)
      annotation (Placement(transformation(extent={{-532,6},{-512,26}})));

    parameter Buildings.DHC.Loads.HotWater.Data.GenericDomesticHotWaterWithHeatExchanger
      datWatHea(VTan=0.1892706, mDom_flow_nominal=6.52944E-06*1000)
      "Data for heat pump water heater with tank"
      annotation (Placement(transformation(extent={{438,-224},{458,-204}})));
    Modelica.Blocks.Sources.Constant conTSetHot(k(
        final unit="K",
        displayUnit="degC") = 313.15)
      "Temperature setpoint for hot water supply to fixture"
      annotation (Placement(transformation(extent={{100,-266},{114,-252}})));
    Buildings.Fluid.Sources.Boundary_pT souCol(
      nPorts=2,
      redeclare package Medium = Medium,
      T(displayUnit="degC"))          "Source of domestic cold water"
      annotation (Placement(transformation(extent={{-17,-17},{17,17}},
          rotation=0,
          origin={109,-183})));
    Modelica.Blocks.Sources.CombiTimeTable sch(
      tableOnFile=true,
      tableName="tab1",
      fileName=Modelica.Utilities.Files.loadResource(
          "modelica://Buildings/Resources/Data/DHC/Loads/HotWater/DHW_ApartmentMidRise.mos"),
      smoothness=Modelica.Blocks.Types.Smoothness.ContinuousDerivative,
      extrapolation=Modelica.Blocks.Types.Extrapolation.Periodic)
      "Domestic hot water fixture draw fraction schedule"
      annotation (Placement(transformation(extent={{-12,-12},{12,12}},
          rotation=180,
          origin={450,-170})));

    Buildings.DHC.Loads.HotWater.ThermostaticMixingValve theMixVal(redeclare
        package Medium = Medium, mMix_flow_nominal=1.2*datWatHea.mDom_flow_nominal)
      annotation (Placement(transformation(extent={{248,-216},{292,-178}})));
    Modelica.Blocks.Sources.Constant conTSetMix(k(
        final unit="K",
        displayUnit="degC") = 308.15)
      "Temperature setpoint for mixed water supply to fixture"
      annotation (Placement(transformation(extent={{-11,-11},{11,11}},
          rotation=180,
          origin={451,-135})));
    Buildings.Electrical.AC.OnePhase.Loads.Inductive acLoad1(mode=Buildings.Electrical.Types.Load.VariableZ_P_input,
        V_nominal=480)
      annotation (Placement(transformation(extent={{23,-22},{-23,22}},
          rotation=270,
          origin={250,-349})));
    Buildings.Electrical.DC.Lines.TwoPortResistance lin(R=0.05)
      "Transmission line"
      annotation (Placement(transformation(extent={{-25,-17},{25,17}},
          rotation=0,
          origin={73,-489})));
    CoSES_ProHMo.Consumer.MyDHW domHotWat(
      redeclare package MediumDom = Buildings.Media.Water,
      redeclare package MediumHea = Buildings.Media.Water,
      dat=datWatHea)
      annotation (Placement(transformation(extent={{144,-304},{208,-232}})));
    Modelica.Blocks.Sources.BooleanStep uHea(startTime(displayUnit="min") =
        60000)
      "On/off signal"
      annotation (Placement(transformation(extent={{-464,-240},{-444,-220}})));
    Modelica.Blocks.Sources.Ramp y(
      height=1,
      duration=1,
      offset=0) "Control signal"
      annotation (Placement(transformation(extent={{-310,84},{-290,104}})));
    Buildings.Fluid.Actuators.Valves.ThreeWayLinear valLin1(
      redeclare package Medium = Buildings.Media.Water,
      l={0.05,0.05},
      m_flow_nominal=2,
      use_inputFilter=false,
      dpValve_nominal=6000,
      energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial)
      "Valve model, linear opening characteristics"
      annotation (Placement(transformation(extent={{-276,-30},{-256,-10}})));
    Buildings.Fluid.Actuators.Valves.ThreeWayLinear valLin2(
      redeclare package Medium = Buildings.Media.Water,
      l={0.05,0.05},
      m_flow_nominal=2,
      use_inputFilter=false,
      dpValve_nominal=6000,
      energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial)
      "Valve model, linear opening characteristics"
      annotation (Placement(transformation(extent={{-224,-88},{-204,-68}})));
    Modelica.Blocks.Sources.RealExpression realExpression1(y=1)
      annotation (Placement(transformation(extent={{-464,-268},{-444,-248}})));
    Buildings.DHC.Loads.BaseClasses.BuildingTimeSeries buiHea(
      have_chiWat=false,
      filNam=
          "modelica://Buildings/Resources/Data/DHC/Loads/Examples/SwissResidential_20190916.mos",
      nPorts_aChiWat=1,
      nPorts_bChiWat=1,
      nPorts_aHeaWat=2,
      nPorts_bHeaWat=2)
      "Building with heating only"
      annotation (Placement(transformation(extent={{170,-60},{244,12}})));

    Buildings.ThermalZones.Detailed.Validation.BaseClasses.SingleZoneFloor sinZonFlo(
      redeclare package Medium =
          Modelica.Media.Water.ConstantPropertyLiquidWater,
      use_windPressure=false,
      gai(K=0*[0.4; 0.4; 0.2]))
      "Single-zone floor model"
      annotation (Placement(transformation(extent={{194,-34},{218,-10}})));
    Generators.Digital_Twins.WolfCGB50_GC wolfCGB50_GC
      annotation (Placement(transformation(extent={{-462,-18},{-396,48}})));
    Generators.Digital_Twins.NeoTower5_GC neoTower5_GC
      annotation (Placement(transformation(extent={{-398,-286},{-336,-224}})));
    Storage.StratifiedHeatStorage tan(
      redeclare package Medium = Buildings.Media.Water,
      m_flow_nominal=0.1,
      VTan=775,
      hTan=3,
      dIns=0.3,
      nSeg=10)
      annotation (Placement(transformation(extent={{-44,-34},{32,42}})));
  equation
    connect(bat.SOC,con. SOC) annotation (Line(
        points={{-176.5,-463.2},{-220,-463.2},{-220,-434},{-213.25,-434}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(con.y,bat. P) annotation (Line(
        points={{-191.375,-434},{-191.375,-436},{-149,-436},{-149,-454}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(bat.terminal,conv. terminal_p) annotation (Line(
        points={{-124,-477},{-44,-477},{-44,-451},{-30,-451}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(acLoad.terminal,gri. terminal) annotation (Line(
        points={{320,-372},{320,-410},{82,-410}},
        color={0,120,120},
        smooth=Smooth.None));
    connect(conv.terminal_n,gri. terminal) annotation (Line(
        points={{12,-451},{82,-451},{82,-410}},
        color={0,120,120},
        smooth=Smooth.None));
    connect(pv.weaBus, weaDat.weaBus) annotation (Line(
        points={{380,-365.5},{380,120},{230,120}},
        color={255,204,51},
        thickness=0.5));
    connect(theMixVal.yMixSet,sch. y[1]) annotation (Line(points={{245.8,-181.8},
            {242,-181.8},{242,-182},{240,-182},{240,-164},{436.8,-164},{436.8,
            -170}},             color={0,0,127}));
    connect(theMixVal.port_col, souCol.ports[1]) annotation (Line(points={{248,
            -212.2},{126,-212.2},{126,-184.7}},                   color={0,127,
            255}));
    connect(conTSetMix.y, theMixVal.TMixSet) annotation (Line(points={{438.9,
            -135},{232,-135},{232,-193.2},{245.8,-193.2}}, color={0,0,127}));
    connect(acLoad1.terminal, gri.terminal) annotation (Line(points={{250,-372},
            {250,-410},{82,-410}}, color={0,120,120}));
    connect(lin.terminal_p, pv.terminal) annotation (Line(points={{98,-489},{
            428,-489},{428,-397},{414,-397}}, color={0,0,255}));
    connect(lin.terminal_n, conv.terminal_p) annotation (Line(points={{48,-489},
            {-44,-489},{-44,-451},{-30,-451}},                       color={0,0,
            255}));
    connect(domHotWat.port_aDom, souCol.ports[2]) annotation (Line(points={{144,
            -246.4},{126,-246.4},{126,-181.3}}, color={0,127,255}));
    connect(domHotWat.port_bDom, theMixVal.port_hot) annotation (Line(points={{
            208,-246.4},{228,-246.4},{228,-204.6},{248,-204.6}}, color={0,127,
            255}));
    connect(conTSetHot.y, domHotWat.TDomSet) annotation (Line(points={{114.7,
            -259},{127.75,-259},{127.75,-268},{140.8,-268}}, color={0,0,127}));
    connect(domHotWat.PEle, acLoad1.Pow) annotation (Line(points={{211.2,-268},
            {211.2,-264},{250,-264},{250,-326}}, color={0,0,127}));
    connect(y.y, valLin1.y) annotation (Line(points={{-289,94},{-266,94},{-266,
            -8}}, color={0,0,127}));
    connect(y.y, valLin2.y) annotation (Line(points={{-289,94},{-216,94},{-216,
            -66},{-214,-66}}, color={0,0,127}));
    connect(buiHea.PPum, acLoad.Pow) annotation (Line(points={{246.467,-16.8},{
            264,-16.8},{264,-168},{320,-168},{320,-328}},
                                color={0,0,127}));
    connect(sinZonFlo.ports[1], buiHea.ports_aHeaWat[1]) annotation (Line(
          points={{198.98,-29.2},{184,-29.2},{184,-32.4},{170,-32.4}},
                        color={0,127,255}));
    connect(sinZonFlo.ports[2], buiHea.ports_bHeaWat[1]) annotation (Line(
          points={{199.58,-29.2},{222,-29.2},{222,-32.4},{244,-32.4}},
                             color={0,127,255}));
    connect(sinZonFlo.weaBus, weaDat.weaBus) annotation (Line(
        points={{198.08,-11.8},{198,-11.8},{198,120},{230,120}},
        color={255,204,51},
        thickness=0.5));
    connect(wolfCGB50_GC.weaBus, weaDat.weaBus) annotation (Line(
        points={{-425.04,55.26},{-425.04,120},{230,120}},
        color={255,204,51},
        thickness=0.5));
    connect(neoTower5_GC.weaBus, weaDat.weaBus) annotation (Line(
        points={{-349.02,-219.04},{-349.02,120},{230,120}},
        color={255,204,51},
        thickness=0.5));
    connect(wolfCGB50_GC.ControlIn, realExpression.y) annotation (Line(points={
            {-464.97,15.99},{-487.985,15.99},{-487.985,16},{-511,16}}, color={0,
            0,127}));
    connect(neoTower5_GC.term_p, gri.terminal) annotation (Line(points={{
            -349.64,-296.54},{-349.64,-410},{82,-410}}, color={0,120,120}));
    connect(wolfCGB50_GC.term_p, gri.terminal) annotation (Line(points={{-409.2,
            -21.3},{-410,-21.3},{-410,-22},{-484,-22},{-484,-410},{82,-410}},
          color={0,120,120}));
    connect(neoTower5_GC.CHPOn, uHea.y) annotation (Line(points={{-392.11,
            -239.19},{-432,-239.19},{-432,-230},{-443,-230}}, color={255,0,255}));
    connect(neoTower5_GC.CHPModulation, realExpression1.y) annotation (Line(
          points={{-392.11,-258.41},{-396,-258.41},{-396,-258},{-443,-258}},
          color={0,0,127}));
    connect(neoTower5_GC.port_a, valLin1.port_3) annotation (Line(points={{
            -328.56,-259.96},{-316,-259.96},{-316,-40},{-266,-40},{-266,-30}},
          color={0,127,255}));
    connect(neoTower5_GC.port_b, valLin2.port_3) annotation (Line(points={{
            -335.38,-241.36},{-214,-241.36},{-214,-88}}, color={0,127,255}));
    connect(wolfCGB50_GC.port_a, valLin1.port_1) annotation (Line(points={{
            -380.16,13.02},{-380.16,4},{-288,4},{-288,-20},{-276,-20}}, color={
            0,127,255}));
    connect(wolfCGB50_GC.port_b, valLin2.port_1) annotation (Line(points={{
            -388.74,37.44},{-232,37.44},{-232,-78},{-224,-78}}, color={0,127,
            255}));
    connect(tan.port_a, valLin1.port_2) annotation (Line(points={{-6,42},{-8,42},
            {-8,56},{-236,56},{-236,-20},{-256,-20}}, color={0,127,255}));
    connect(tan.port_b, buiHea.ports_aHeaWat[2]) annotation (Line(points={{-6,
            -34},{-8,-34},{-8,-48},{160,-48},{160,-30},{170,-30}}, color={0,127,
            255}));
    connect(domHotWat.port_bHea, tan.port_b) annotation (Line(points={{144,
            -289.6},{78,-289.6},{78,-36},{-6,-36},{-6,-34}}, color={0,127,255}));
    connect(tan.fluPorVol[6], domHotWat.port_aHea) annotation (Line(points={{
            -25,4.76},{-25,10},{-58,10},{-58,-312},{220,-312},{220,-289.6},{208,
            -289.6}}, color={0,127,255}));
    connect(valLin2.port_2, tan.fluPorVol[2]) annotation (Line(points={{-204,
            -78},{-204,-80},{-162,-80},{-162,-10},{-26,-10},{-26,-1.32},{-25,
            -1.32}}, color={0,127,255}));
    connect(buiHea.ports_bHeaWat[2], tan.fluPorVol[4]) annotation (Line(points=
            {{244,-30},{256,-30},{256,-84},{-68,-84},{-68,-2},{-26,-2},{-26,
            1.72},{-25,1.72}}, color={0,127,255}));
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
        StartTime=12000,
        StopTime=90000,
        Tolerance=1e-06,
        __Dymola_Algorithm="Dassl"),
      Documentation(info="<html>
<p>
This model provides an example for a building with loads provided
as time series and connected to a direct ETS for heating with the
return heating water temperature controlled below a maximum threshold.
</p>
</html>",   revisions="<html>
<ul>
<li>
March 20, 2022, by Chengnan Shi:<br/>
First implementation.
</li>
</ul>
</html>"));
  end MF5;

  package Experiments
    model HeatingBuildingTimeSeries_NeoTower2_GC
      "Example model of a building with loads provided as time series"
      extends Modelica.Icons.Example;
      package Medium=Buildings.Media.Water
        "Medium model";
      Buildings.Electrical.DC.Sources.PVSimpleOriented
                                             pv(
        A=200e3/800/0.12,
        til=0.34906585039887,
        azi=-0.78539816339745,
        V_nominal=480) "PV array"
        annotation (Placement(transformation(extent={{144,-154},{124,-134}})));
      Buildings.Electrical.DC.Storage.Battery bat(EMax=500e3*4*3600, V_nominal=
            480)       "Battery"
        annotation (Placement(transformation(extent={{272,-188},{222,-142}})));
      Buildings.Electrical.AC.OnePhase.Conversion.ACDCConverter conv(
          conversionFactor=480/480, eta=0.9)
                 "AC/DC converter"
        annotation (Placement(transformation(extent={{176,-302},{134,-256}})));
      Buildings.Electrical.AC.OnePhase.Sources.Grid gri(
        f=60,
        V=480,
        phiSou=0) annotation (Placement(transformation(extent={{-4,-220},{36,-180}})));
      Buildings.Examples.ChillerPlant.BaseClasses.Controls.BatteryControl
                                          con "Battery controller"
        annotation (Placement(transformation(extent={{184,-132},{204,-112}})));
      Buildings.Electrical.AC.OnePhase.Loads.Inductive acLoad(mode=Buildings.Electrical.Types.Load.VariableZ_P_input,
          V_nominal=480)
        annotation (Placement(transformation(extent={{22,-20},{-22,20}},
            rotation=270,
            origin={88,-134})));
      CoSES_ProHMo.Storage.MyStratified tan(
        redeclare package Medium = Buildings.Media.Water,
        m_flow_nominal=0.1,
        VTan(displayUnit="l") = 0.775,
        hTan=3,
        dIns=0.3,
        nSeg=10) annotation (Placement(transformation(extent={{-146,-46},{-74,26}})));
      Buildings.BoundaryConditions.WeatherData.ReaderTMY3 weaDat(filNam=
            ModelicaServices.ExternalReferences.loadResource(
            "modelica://Buildings/Resources/weatherdata/USA_CA_San.Francisco.Intl.AP.724940_TMY3.mos"))
        annotation (Placement(transformation(extent={{156,90},{176,110}})));
      Modelica.Blocks.Sources.RealExpression realExpression(y=1)
        annotation (Placement(transformation(extent={{-488,-66},{-468,-46}})));
      Test_prosumer1.TESTMyNeoTower2_GC tESTMyNeoTower2_GC
        annotation (Placement(transformation(extent={{-400,-66},{-332,2}})));
      Modelica.Blocks.Sources.BooleanExpression booleanExpression(y=true)
        annotation (Placement(transformation(extent={{-488,-22},{-468,-2}})));
      Buildings.DHC.Loads.BaseClasses.BuildingTimeSeries buiHea(
        have_chiWat=false,
        filNam=
            "modelica://Buildings/Resources/Data/DHC/Loads/Examples/SwissResidential_20190916.mos",
        nPorts_aChiWat=1,
        nPorts_bChiWat=1,
        nPorts_aHeaWat=1,
        nPorts_bHeaWat=1)
        "Building with heating only"
        annotation (Placement(transformation(extent={{32,-58},{62,-14}})));

    equation
      connect(bat.SOC,con. SOC) annotation (Line(
          points={{219.5,-151.2},{176,-151.2},{176,-122},{182.75,-122}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(con.y,bat. P) annotation (Line(
          points={{204.625,-122},{247,-122},{247,-142}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(pv.terminal,conv. terminal_p) annotation (Line(
          points={{144,-144},{134,-144},{134,-279}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(bat.terminal,conv. terminal_p) annotation (Line(
          points={{272,-165},{284,-165},{284,-218},{134,-218},{134,-279}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(acLoad.terminal,gri. terminal) annotation (Line(
          points={{88,-156},{88,-232},{16,-232},{16,-220}},
          color={0,120,120},
          smooth=Smooth.None));
      connect(conv.terminal_n,gri. terminal) annotation (Line(
          points={{176,-279},{188,-279},{188,-316},{16,-316},{16,-220}},
          color={0,120,120},
          smooth=Smooth.None));
      connect(pv.weaBus, weaDat.weaBus) annotation (Line(
          points={{134,-135},{134,100},{176,100}},
          color={255,204,51},
          thickness=0.5));
      connect(tESTMyNeoTower2_GC.CHPModulation, realExpression.y) annotation (
          Line(points={{-388.312,-12.5273},{-460,-12.5273},{-460,-56},{-467,-56}},
            color={0,0,127}));
      connect(booleanExpression.y, tESTMyNeoTower2_GC.CHPOn) annotation (Line(
            points={{-467,-12},{-412,-12},{-412,7.87273},{-388.312,7.87273}},
                                                                          color={
              255,0,255}));
      connect(gri.terminal, tESTMyNeoTower2_GC.term_p) annotation (Line(points={{16,-220},
              {16,-232},{-358.35,-232},{-358.35,-51.7818}},     color={0,120,120}));
      connect(tan.port_a, tESTMyNeoTower2_GC.port_a) annotation (Line(points={{-110,26},
              {-112,26},{-112,40},{-312,40},{-312,-39.4182},{-344.75,-39.4182}},
                               color={0,127,255}));
      connect(weaDat.weaBus, tESTMyNeoTower2_GC.weaBus) annotation (Line(
          points={{176,100},{-362,100},{-362,10},{-357.925,10},{-357.925,25.4909}},
          color={255,204,51},
          thickness=0.5));
      connect(tan.port_b, buiHea.ports_aHeaWat[1]) annotation (Line(points={{-110,
              -46},{-58,-46},{-58,-32},{32,-32},{32,-40.4}},
            color={0,127,255}));
      connect(buiHea.PPum, acLoad.Pow) annotation (Line(points={{63,-31.6},{88,
              -31.6},{88,-112}}, color={0,0,127}));
      connect(buiHea.ports_bHeaWat[1], tan.fluPorVol[4]) annotation (Line(points={{62,
              -40.4},{62,-68},{-160,-68},{-160,-12.16},{-128,-12.16}},
                        color={0,127,255}));
      connect(tESTMyNeoTower2_GC.port_b, tan.fluPorVol[2]) annotation (Line(
            points={{-345.6,-20.2545},{-160,-20.2545},{-160,-15.04},{-128,-15.04}},
            color={0,127,255}));
      annotation (
        Icon(
          coordinateSystem(
            preserveAspectRatio=false, extent={{-480,-320},{280,120}})),
        Diagram(
            coordinateSystem(
            preserveAspectRatio=false, extent={{-480,-320},{280,120}})),
        __Dymola_Commands(
          file="modelica://Buildings/Resources/Scripts/Dymola/DHC/Loads/Heating/Examples/BuildingTimeSeriesWithETS.mos" "Simulate and plot"),
        experiment(
          StartTime=2592000,
          StopTime=3628800,
          Tolerance=1e-06),
        Documentation(info="<html>
<p>
This model provides an example for a building with loads provided
as time series and connected to a direct ETS for heating with the
return heating water temperature controlled below a maximum threshold.
</p>
</html>",     revisions="<html>
<ul>
<li>
March 20, 2022, by Chengnan Shi:<br/>
First implementation.
</li>
</ul>
</html>"));
    end HeatingBuildingTimeSeries_NeoTower2_GC;

    model HeatingBuildingTimeSeries_WolfCGB14
      "Example model of a building with loads provided as time series"
      extends Modelica.Icons.Example;
      package Medium=Buildings.Media.Water
        "Medium model";
      Buildings.Electrical.DC.Sources.PVSimpleOriented
                                             pv(
        A=200e3/800/0.12,
        til=0.34906585039887,
        azi=-0.78539816339745,
        V_nominal=480) "PV array"
        annotation (Placement(transformation(extent={{332,-328},{312,-308}})));
      Buildings.Electrical.DC.Storage.Battery bat(EMax=500e3*4*3600, V_nominal=
            480)       "Battery"
        annotation (Placement(transformation(extent={{460,-362},{410,-316}})));
      Buildings.Electrical.AC.OnePhase.Conversion.ACDCConverter conv(
          conversionFactor=480/480, eta=0.9)
                 "AC/DC converter"
        annotation (Placement(transformation(extent={{364,-476},{322,-430}})));
      Buildings.Electrical.AC.OnePhase.Sources.Grid gri(
        f=60,
        V=480,
        phiSou=0) annotation (Placement(transformation(extent={{182,-380},{222,
                -340}})));
      Buildings.Examples.ChillerPlant.BaseClasses.Controls.BatteryControl
                                          con "Battery controller"
        annotation (Placement(transformation(extent={{372,-306},{392,-286}})));
      Buildings.Electrical.AC.OnePhase.Loads.Inductive acLoad(mode=Buildings.Electrical.Types.Load.VariableZ_P_input,
          V_nominal=480)
        annotation (Placement(transformation(extent={{22,-20},{-22,20}},
            rotation=270,
            origin={276,-308})));
      CoSES_ProHMo.Storage.MyStratified tan(
        redeclare package Medium = Buildings.Media.Water,
        m_flow_nominal=0.1,
        VTan(displayUnit="l") = 0.775,
        hTan=3,
        dIns=0.3,
        nSeg=10) annotation (Placement(transformation(extent={{-144,-36},{-72,36}})));
      Buildings.BoundaryConditions.WeatherData.ReaderTMY3 weaDat(filNam=
            ModelicaServices.ExternalReferences.loadResource(
            "modelica://Buildings/Resources/weatherdata/USA_CA_San.Francisco.Intl.AP.724940_TMY3.mos"))
        annotation (Placement(transformation(extent={{156,90},{176,110}})));
      Modelica.Blocks.Sources.RealExpression realExpression(y=1)
        annotation (Placement(transformation(extent={{-448,-32},{-428,-12}})));
      Test_prosumer1.TestMyWolfCGB14_GC testMyWolfCGB14_GC
        annotation (Placement(transformation(extent={{-404,-56},{-338,10}})));
      Buildings.DHC.Loads.BaseClasses.BuildingTimeSeries buiHea(
        have_chiWat=false,
        filNam=
            "modelica://Buildings/Resources/Data/DHC/Loads/Examples/SwissResidential_20190916.mos",
        nPorts_aChiWat=1,
        nPorts_bChiWat=1,
        nPorts_aHeaWat=1,
        nPorts_bHeaWat=1)
        "Building with heating only"
        annotation (Placement(transformation(extent={{192,-74},{236,-16}})));

    equation
      connect(bat.SOC,con. SOC) annotation (Line(
          points={{407.5,-325.2},{360,-325.2},{360,-296},{370.75,-296}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(con.y,bat. P) annotation (Line(
          points={{392.625,-296},{435,-296},{435,-316}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(pv.terminal,conv. terminal_p) annotation (Line(
          points={{332,-318},{332,-320},{340,-320},{340,-420},{312,-420},{312,
              -453},{322,-453}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(bat.terminal,conv. terminal_p) annotation (Line(
          points={{460,-339},{472,-339},{472,-420},{312,-420},{312,-453},{322,
              -453}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(acLoad.terminal,gri. terminal) annotation (Line(
          points={{276,-330},{276,-392},{202,-392},{202,-380}},
          color={0,120,120},
          smooth=Smooth.None));
      connect(conv.terminal_n,gri. terminal) annotation (Line(
          points={{364,-453},{384,-453},{384,-512},{202,-512},{202,-380}},
          color={0,120,120},
          smooth=Smooth.None));
      connect(pv.weaBus, weaDat.weaBus) annotation (Line(
          points={{322,-309},{322,100},{176,100}},
          color={255,204,51},
          thickness=0.5));
      connect(testMyWolfCGB14_GC.port_a, tan.port_a) annotation (Line(points={{
              -344.253,-30.8},{-302,-30.8},{-302,48},{-108,48},{-108,36}},  color
            ={0,127,255}));
      connect(testMyWolfCGB14_GC.ControlIn, realExpression.y) annotation (Line(
            points={{-397.226,-31.7},{-397.226,-22},{-427,-22}},color={0,0,127}));
      connect(testMyWolfCGB14_GC.term_p, gri.terminal) annotation (Line(points={{
              -369.263,-65},{-369.263,-392},{202,-392},{202,-380}},
                                                                color={0,120,120}));
      connect(testMyWolfCGB14_GC.weaBus, weaDat.weaBus) annotation (Line(
          points={{-377.6,4.6},{-366,4.6},{-366,100},{176,100}},
          color={255,204,51},
          thickness=0.5));
      connect(buiHea.PPum, acLoad.Pow) annotation (Line(points={{237.467,-39.2},
              {276,-39.2},{276,-286}},
                                  color={0,0,127}));
      connect(testMyWolfCGB14_GC.port_b, tan.fluPorVol[2]) annotation (Line(
            points={{-348.768,-6.2},{-130,-6.2},{-130,-6},{-128,-6},{-128,-5.04},
              {-126,-5.04}}, color={0,127,255}));
      connect(tan.port_b, buiHea.ports_aHeaWat[1]) annotation (Line(points={{-108,
              -36},{-108,-50.8},{192,-50.8}}, color={0,127,255}));
      connect(buiHea.ports_bHeaWat[1], tan.fluPorVol[4]) annotation (Line(points={{236,
              -50.8},{248,-50.8},{248,-96},{-160,-96},{-160,-2.16},{-126,-2.16}},
                       color={0,127,255}));
      annotation (
        Icon(
          coordinateSystem(
            preserveAspectRatio=false, extent={{-400,-520},{440,120}})),
        Diagram(
            coordinateSystem(
            preserveAspectRatio=false, extent={{-400,-520},{440,120}})),
        __Dymola_Commands(
          file="modelica://Buildings/Resources/Scripts/Dymola/DHC/Loads/Heating/Examples/BuildingTimeSeriesWithETS.mos" "Simulate and plot"),
        experiment(
          StartTime=2592000,
          StopTime=3628800,
          Tolerance=1e-06),
        Documentation(info="<html>
<p>
This model provides an example for a building with loads provided
as time series and connected to a direct ETS for heating with the
return heating water temperature controlled below a maximum threshold.
</p>
</html>",     revisions="<html>
<ul>
<li>
March 20, 2022, by Chengnan Shi:<br/>
First implementation.
</li>
</ul>
</html>"));
    end HeatingBuildingTimeSeries_WolfCGB14;

    model HeatingBuildingTimeSeries_WolfCGB14_NeoTower_GC
      "Example model of a building with loads provided as time series"
      extends Modelica.Icons.Example;
      package Medium=Buildings.Media.Water
        "Medium model";
      Buildings.Electrical.DC.Sources.PVSimpleOriented
                                             pv(
        A=200e3/800/0.12,
        til=0.34906585039887,
        azi=-0.78539816339745,
        V_nominal=480) "PV array"
        annotation (Placement(transformation(extent={{332,-328},{312,-308}})));
      Buildings.Electrical.DC.Storage.Battery bat(EMax=500e3*4*3600, V_nominal=
            480)       "Battery"
        annotation (Placement(transformation(extent={{460,-362},{410,-316}})));
      Buildings.Electrical.AC.OnePhase.Conversion.ACDCConverter conv(
          conversionFactor=480/480, eta=0.9)
                 "AC/DC converter"
        annotation (Placement(transformation(extent={{364,-476},{322,-430}})));
      Buildings.Electrical.AC.OnePhase.Sources.Grid gri(
        f=60,
        V=480,
        phiSou=0) annotation (Placement(transformation(extent={{182,-380},{222,
                -340}})));
      Buildings.Examples.ChillerPlant.BaseClasses.Controls.BatteryControl
                                          con "Battery controller"
        annotation (Placement(transformation(extent={{372,-306},{392,-286}})));
      Buildings.Electrical.AC.OnePhase.Loads.Inductive acLoad(mode=Buildings.Electrical.Types.Load.VariableZ_P_input,
          V_nominal=480)
        annotation (Placement(transformation(extent={{22,-20},{-22,20}},
            rotation=270,
            origin={276,-308})));
      CoSES_ProHMo.Storage.MyStratified tan(
        redeclare package Medium = Buildings.Media.Water,
        m_flow_nominal=0.1,
        VTan(displayUnit="l") = 0.775,
        hTan=3,
        dIns=0.3,
        nSeg=10) annotation (Placement(transformation(extent={{-144,-36},{-72,36}})));
      Buildings.BoundaryConditions.WeatherData.ReaderTMY3 weaDat(filNam=
            ModelicaServices.ExternalReferences.loadResource(
            "modelica://Buildings/Resources/weatherdata/USA_CA_San.Francisco.Intl.AP.724940_TMY3.mos"))
        annotation (Placement(transformation(extent={{208,114},{228,134}})));
      Modelica.Blocks.Sources.RealExpression realExpression(y=1)
        annotation (Placement(transformation(extent={{-442,6},{-422,26}})));
      Test_prosumer1.TestMyWolfCGB14_GC testMyWolfCGB14_GC
        annotation (Placement(transformation(extent={{-406,-18},{-340,48}})));
      Test_prosumer1.TESTMyNeoTower2_GC tESTMyNeoTower2_GC
        annotation (Placement(transformation(extent={{-490,-164},{-422,-96}})));
      Modelica.Blocks.Sources.RealExpression realExpression1(y=1)
        annotation (Placement(transformation(extent={{-538,-166},{-518,-146}})));
      Modelica.Blocks.Sources.BooleanExpression booleanExpression(y=true)
        annotation (Placement(transformation(extent={{-538,-122},{-518,-102}})));
      Buildings.Fluid.Actuators.Valves.ThreeWayLinear valLin(
        redeclare package Medium = Buildings.Media.Water,
        l={0.05,0.05},
        m_flow_nominal=2,
        use_inputFilter=false,
        dpValve_nominal=6000,
        energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial)
        "Valve model, linear opening characteristics"
        annotation (Placement(transformation(extent={{-172,-148},{-152,-128}})));
      Buildings.Fluid.Actuators.Valves.ThreeWayLinear valLin1(
        redeclare package Medium = Buildings.Media.Water,
        l={0.05,0.05},
        m_flow_nominal=2,
        use_inputFilter=false,
        dpValve_nominal=6000,
        energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial)
        "Valve model, linear opening characteristics"
        annotation (Placement(transformation(extent={{-244,-26},{-224,-6}})));
      Modelica.Blocks.Sources.Ramp y(
        height=1,
        duration=1,
        offset=0) "Control signal"
        annotation (Placement(transformation(extent={{-298,68},{-278,88}})));
      Buildings.DHC.Loads.BaseClasses.BuildingTimeSeries buiHea(
        have_chiWat=false,
        filNam=
            "modelica://Buildings/Resources/Data/DHC/Loads/Examples/SwissResidential_20190916.mos",
        nPorts_aChiWat=1,
        nPorts_bChiWat=1,
        nPorts_aHeaWat=1,
        nPorts_bHeaWat=1)
        "Building with heating only"
        annotation (Placement(transformation(extent={{118,-56},{162,2}})));

    equation
      connect(bat.SOC,con. SOC) annotation (Line(
          points={{407.5,-325.2},{360,-325.2},{360,-296},{370.75,-296}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(con.y,bat. P) annotation (Line(
          points={{392.625,-296},{435,-296},{435,-316}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(pv.terminal,conv. terminal_p) annotation (Line(
          points={{332,-318},{332,-320},{340,-320},{340,-420},{312,-420},{312,
              -453},{322,-453}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(bat.terminal,conv. terminal_p) annotation (Line(
          points={{460,-339},{472,-339},{472,-420},{312,-420},{312,-453},{322,
              -453}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(acLoad.terminal,gri. terminal) annotation (Line(
          points={{276,-330},{276,-392},{202,-392},{202,-380}},
          color={0,120,120},
          smooth=Smooth.None));
      connect(conv.terminal_n,gri. terminal) annotation (Line(
          points={{364,-453},{384,-453},{384,-512},{202,-512},{202,-380}},
          color={0,120,120},
          smooth=Smooth.None));
      connect(pv.weaBus, weaDat.weaBus) annotation (Line(
          points={{322,-309},{322,124},{228,124}},
          color={255,204,51},
          thickness=0.5));
      connect(testMyWolfCGB14_GC.ControlIn, realExpression.y) annotation (Line(
            points={{-399.226,6.3},{-413.665,6.3},{-413.665,16},{-421,16}},
                                                                color={0,0,127}));
      connect(testMyWolfCGB14_GC.term_p, gri.terminal) annotation (Line(points={{
              -371.263,-27},{-371.263,-392},{202,-392},{202,-380}},
                                                                color={0,120,120}));
      connect(tESTMyNeoTower2_GC.CHPOn, booleanExpression.y) annotation (Line(
            points={{-478.312,-90.1273},{-478.312,-112},{-517,-112}},
                                                                 color={255,0,255}));
      connect(tESTMyNeoTower2_GC.CHPModulation, realExpression1.y) annotation (
          Line(points={{-478.312,-110.527},{-504,-110.527},{-504,-156},{-517,
              -156}},
            color={0,0,127}));
      connect(tESTMyNeoTower2_GC.term_p, gri.terminal) annotation (Line(points={{-448.35,
              -149.782},{-352,-149.782},{-352,-392},{202,-392},{202,-380}},
            color={0,120,120}));
      connect(testMyWolfCGB14_GC.port_a, valLin1.port_1) annotation (Line(points={{
              -346.253,7.2},{-346.253,-16},{-244,-16}},   color={0,127,255}));
      connect(tESTMyNeoTower2_GC.port_a, valLin1.port_3) annotation (Line(points={{-434.75,
              -137.418},{-234,-137.418},{-234,-26}},        color={0,127,255}));
      connect(tan.port_a, valLin1.port_2) annotation (Line(points={{-108,36},{
              -108,48},{-212,48},{-212,-16},{-224,-16}},  color={0,127,255}));
      connect(valLin.port_3, tESTMyNeoTower2_GC.port_b) annotation (Line(points={{-162,
              -148},{-164,-148},{-164,-156},{-404,-156},{-404,-118.255},{-435.6,
              -118.255}},        color={0,127,255}));
      connect(valLin.port_1, testMyWolfCGB14_GC.port_b) annotation (Line(points={{-172,
              -138},{-172,-140},{-314,-140},{-314,-106},{-350.768,-106},{
              -350.768,31.8}}, color={0,127,255}));
      connect(y.y, valLin1.y) annotation (Line(points={{-277,78},{-234,78},{-234,
              -4}}, color={0,0,127}));
      connect(y.y, valLin.y) annotation (Line(points={{-277,78},{-236,78},{-236,8},
              {-162,8},{-162,-126}}, color={0,0,127}));
      connect(testMyWolfCGB14_GC.weaBus, weaDat.weaBus) annotation (Line(
          points={{-379.6,42.6},{-379.6,124},{228,124}},
          color={255,204,51},
          thickness=0.5));
      connect(tESTMyNeoTower2_GC.weaBus, weaDat.weaBus) annotation (Line(
          points={{-447.925,-72.5091},{-447.925,124},{228,124}},
          color={255,204,51},
          thickness=0.5));
      connect(buiHea.ports_aHeaWat[1], tan.port_b) annotation (Line(points={{118,
              -32.8},{-60,-32.8},{-60,-48},{-108,-48},{-108,-36}}, color={0,127,
              255}));
      connect(tan.fluPorVol[4], buiHea.ports_bHeaWat[1]) annotation (Line(points={{-126,
              -2.16},{-156,-2.16},{-156,-72},{-144,-72},{-144,-80},{176,-80},{
              176,-32.8},{162,-32.8}},
                       color={0,127,255}));
      connect(valLin.port_2, tan.fluPorVol[2]) annotation (Line(points={{-152,
              -138},{-152,-140},{-144,-140},{-144,-72},{-156,-72},{-156,-5.04},{
              -126,-5.04}}, color={0,127,255}));
      connect(buiHea.PPum, acLoad.Pow) annotation (Line(points={{163.467,-21.2},
              {276,-21.2},{276,-286}},
                                  color={0,0,127}));
      annotation (
        Icon(
          coordinateSystem(
            preserveAspectRatio=false, extent={{-400,-520},{440,120}})),
        Diagram(
            coordinateSystem(
            preserveAspectRatio=false, extent={{-400,-520},{440,120}})),
        __Dymola_Commands(
          file="modelica://Buildings/Resources/Scripts/Dymola/DHC/Loads/Heating/Examples/BuildingTimeSeriesWithETS.mos" "Simulate and plot"),
        experiment(
          StartTime=2592000,
          StopTime=3628800,
          Tolerance=1e-06),
        Documentation(info="<html>
<p>
This model provides an example for a building with loads provided
as time series and connected to a direct ETS for heating with the
return heating water temperature controlled below a maximum threshold.
</p>
</html>",     revisions="<html>
<ul>
<li>
March 20, 2022, by Chengnan Shi:<br/>
First implementation.
</li>
</ul>
</html>"));
    end HeatingBuildingTimeSeries_WolfCGB14_NeoTower_GC;

    model HeatingBuildingTimeSeriesWithETS_Solar
      "Example model of a building with loads provided as time series"
      extends Modelica.Icons.Example;
      package Medium=Buildings.Media.Water
        "Medium model";
      Buildings.Electrical.DC.Sources.PVSimpleOriented
                                             pv(
        A=200e3/800/0.12,
        til=0.34906585039887,
        azi=-0.78539816339745,
        V_nominal=480) "PV array"
        annotation (Placement(transformation(extent={{144,-154},{124,-134}})));
      Buildings.Electrical.DC.Storage.Battery bat(EMax=500e3*4*3600, V_nominal=
            480)       "Battery"
        annotation (Placement(transformation(extent={{272,-188},{222,-142}})));
      Buildings.Electrical.AC.OnePhase.Conversion.ACDCConverter conv(
          conversionFactor=480/480, eta=0.9)
                 "AC/DC converter"
        annotation (Placement(transformation(extent={{176,-302},{134,-256}})));
      Buildings.Electrical.AC.OnePhase.Sources.Grid gri(
        f=60,
        V=480,
        phiSou=0) annotation (Placement(transformation(extent={{-6,-206},{34,-166}})));
      Buildings.Examples.ChillerPlant.BaseClasses.Controls.BatteryControl
                                          con "Battery controller"
        annotation (Placement(transformation(extent={{184,-132},{204,-112}})));
      Buildings.Electrical.AC.OnePhase.Loads.Inductive acLoad(mode=Buildings.Electrical.Types.Load.VariableZ_P_input,
          V_nominal=480)
        annotation (Placement(transformation(extent={{22,-20},{-22,20}},
            rotation=270,
            origin={90,-134})));
      CoSES_ProHMo.Storage.MyStratified tan(
        redeclare package Medium = Buildings.Media.Water,
        m_flow_nominal=0.1,
        VTan(displayUnit="l") = 0.775,
        hTan=3,
        dIns=0.3,
        nSeg=10) annotation (Placement(transformation(extent={{-146,-46},{-74,26}})));
      Buildings.BoundaryConditions.WeatherData.ReaderTMY3 weaDat(filNam=
            ModelicaServices.ExternalReferences.loadResource(
            "modelica://Buildings/Resources/weatherdata/USA_CA_San.Francisco.Intl.AP.724940_TMY3.mos"))
        annotation (Placement(transformation(extent={{156,90},{176,110}})));
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
        annotation (Placement(transformation(extent={{-420,8},{-360,62}})));

      Test_prosumer1.Test_Pump_controler testClosedLoop_Pump
        annotation (Placement(transformation(extent={{-322,-92},{-302,-70}})));
      Modelica.Blocks.Sources.Pulse y(
        offset=0.25,
        startTime=0,
        amplitude=0.5,
        period=15*60) "Input signal"
                     annotation (Placement(transformation(extent={{-366,-54},{
                -346,-34}})));
      Buildings.DHC.Loads.BaseClasses.BuildingTimeSeries buiHea(
        have_chiWat=false,
        filNam=
            "modelica://Buildings/Resources/Data/DHC/Loads/Examples/SwissResidential_20190916.mos",
        nPorts_aChiWat=1,
        nPorts_bChiWat=1,
        nPorts_aHeaWat=1,
        nPorts_bHeaWat=1)
        "Building with heating only"
        annotation (Placement(transformation(extent={{36,-84},{78,-26}})));

    equation
      connect(bat.SOC,con. SOC) annotation (Line(
          points={{219.5,-151.2},{176,-151.2},{176,-122},{182.75,-122}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(con.y,bat. P) annotation (Line(
          points={{204.625,-122},{247,-122},{247,-142}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(pv.terminal,conv. terminal_p) annotation (Line(
          points={{144,-144},{134,-144},{134,-279}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(bat.terminal,conv. terminal_p) annotation (Line(
          points={{272,-165},{284,-165},{284,-218},{134,-218},{134,-279}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(acLoad.terminal,gri. terminal) annotation (Line(
          points={{90,-156},{90,-206},{14,-206}},
          color={0,120,120},
          smooth=Smooth.None));
      connect(conv.terminal_n,gri. terminal) annotation (Line(
          points={{176,-279},{188,-279},{188,-316},{14,-316},{14,-206}},
          color={0,120,120},
          smooth=Smooth.None));
      connect(pv.weaBus, weaDat.weaBus) annotation (Line(
          points={{134,-135},{134,100},{176,100}},
          color={255,204,51},
          thickness=0.5));
      connect(tan.port_a, solCol.port_b) annotation (Line(points={{-110,26},{-112,
              26},{-112,35},{-360,35}}, color={0,127,255}));
      connect(weaDat.weaBus, solCol.weaBus) annotation (Line(
          points={{176,100},{112,100},{112,98},{-420,98},{-420,60.92}},
          color={255,204,51},
          thickness=0.5));
      connect(solCol.port_a, testClosedLoop_Pump.port_a) annotation (Line(points=
              {{-420,35},{-512,35},{-512,-80.78},{-321.6,-80.78}}, color={0,127,
              255}));
      connect(y.y, testClosedLoop_Pump.SetInSignal) annotation (Line(points={{
              -345,-44},{-320.2,-44},{-320.2,-68.9}}, color={0,0,127}));
      connect(buiHea.ports_aHeaWat[1], tan.port_b) annotation (Line(points={{36,
              -60.8},{-64,-60.8},{-64,-60},{-110,-60},{-110,-46}},
                                               color={0,127,255}));
      connect(buiHea.ports_bHeaWat[1], tan.fluPorVol[4]) annotation (Line(points={{78,
              -60.8},{96,-60.8},{96,-100},{-152,-100},{-152,-12.16},{-128,
              -12.16}}, color={0,127,255}));
      connect(buiHea.PPum, acLoad.Pow) annotation (Line(points={{79.4,-49.2},{
              84,-49.2},{84,-96},{90,-96},{90,-112}},
                                 color={0,0,127}));
      connect(testClosedLoop_Pump.port_b, tan.fluPorVol[2]) annotation (Line(
            points={{-301.6,-80.78},{-160,-80.78},{-160,-15.04},{-128,-15.04}},
            color={0,127,255}));
      connect(testClosedLoop_Pump.term_p, gri.terminal) annotation (Line(points={
              {-306.4,-95.74},{-306.4,-206},{14,-206}}, color={0,120,120}));
      annotation (
        Icon(
          coordinateSystem(
            preserveAspectRatio=false, extent={{-500,-340},{300,140}})),
        Diagram(
            coordinateSystem(
            preserveAspectRatio=false, extent={{-500,-340},{300,140}})),
        __Dymola_Commands(
          file="modelica://Buildings/Resources/Scripts/Dymola/DHC/Loads/Heating/Examples/BuildingTimeSeriesWithETS.mos" "Simulate and plot"),
        experiment(
          StartTime=2592000,
          StopTime=3628800,
          Tolerance=1e-06),
        Documentation(info="<html>
<p>
This model provides an example for a building with loads provided
as time series and connected to a direct ETS for heating with the
return heating water temperature controlled below a maximum threshold.
</p>
</html>",     revisions="<html>
<ul>
<li>
March 20, 2022, by Chengnan Shi:<br/>
First implementation.
</li>
</ul>
</html>"));
    end HeatingBuildingTimeSeriesWithETS_Solar;

    model HeatingBuildingTimeSeries_WolfCGB14_NeoTower_GC_Solar
      "Example model of a building with loads provided as time series"
      extends Modelica.Icons.Example;
      package Medium=Buildings.Media.Water
        "Medium model";
      Buildings.Electrical.DC.Sources.PVSimpleOriented
                                             pv(
        A=200e3/800/0.12,
        til=0.34906585039887,
        azi=-0.78539816339745,
        V_nominal=480) "PV array"
        annotation (Placement(transformation(extent={{332,-328},{312,-308}})));
      Buildings.Electrical.DC.Storage.Battery bat(EMax=500e3*4*3600, V_nominal=
            480)       "Battery"
        annotation (Placement(transformation(extent={{460,-362},{410,-316}})));
      Buildings.Electrical.AC.OnePhase.Conversion.ACDCConverter conv(
          conversionFactor=480/480, eta=0.9)
                 "AC/DC converter"
        annotation (Placement(transformation(extent={{364,-476},{322,-430}})));
      Buildings.Electrical.AC.OnePhase.Sources.Grid gri(
        f=60,
        V=480,
        phiSou=0) annotation (Placement(transformation(extent={{182,-380},{222,
                -340}})));
      Buildings.Examples.ChillerPlant.BaseClasses.Controls.BatteryControl
                                          con "Battery controller"
        annotation (Placement(transformation(extent={{372,-306},{392,-286}})));
      Buildings.Electrical.AC.OnePhase.Loads.Inductive acLoad(mode=Buildings.Electrical.Types.Load.VariableZ_P_input,
          V_nominal=480)
        annotation (Placement(transformation(extent={{22,-20},{-22,20}},
            rotation=270,
            origin={276,-308})));
      CoSES_ProHMo.Storage.MyStratified tan(
        redeclare package Medium = Buildings.Media.Water,
        m_flow_nominal=0.1,
        VTan(displayUnit="l") = 0.775,
        hTan=3,
        dIns=0.3,
        nSeg=10) annotation (Placement(transformation(extent={{-144,-36},{-72,36}})));
      Buildings.BoundaryConditions.WeatherData.ReaderTMY3 weaDat(filNam=
            ModelicaServices.ExternalReferences.loadResource(
            "modelica://Buildings/Resources/weatherdata/USA_CA_San.Francisco.Intl.AP.724940_TMY3.mos"))
        annotation (Placement(transformation(extent={{208,114},{228,134}})));
      Modelica.Blocks.Sources.RealExpression realExpression(y=1)
        annotation (Placement(transformation(extent={{-442,6},{-422,26}})));
      Test_prosumer1.TestMyWolfCGB14_GC testMyWolfCGB14_GC
        annotation (Placement(transformation(extent={{-406,-18},{-340,48}})));
      Test_prosumer1.TESTMyNeoTower2_GC tESTMyNeoTower2_GC
        annotation (Placement(transformation(extent={{-490,-164},{-422,-96}})));
      Modelica.Blocks.Sources.RealExpression realExpression1(y=1)
        annotation (Placement(transformation(extent={{-538,-166},{-518,-146}})));
      Modelica.Blocks.Sources.BooleanExpression booleanExpression(y=true)
        annotation (Placement(transformation(extent={{-538,-122},{-518,-102}})));
      Buildings.Fluid.Actuators.Valves.ThreeWayLinear valLin(
        redeclare package Medium = Buildings.Media.Water,
        l={0.05,0.05},
        m_flow_nominal=2,
        use_inputFilter=false,
        dpValve_nominal=6000,
        energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial)
        "Valve model, linear opening characteristics"
        annotation (Placement(transformation(extent={{-172,-148},{-152,-128}})));
      Buildings.Fluid.Actuators.Valves.ThreeWayLinear valLin1(
        redeclare package Medium = Buildings.Media.Water,
        l={0.05,0.05},
        m_flow_nominal=2,
        use_inputFilter=false,
        dpValve_nominal=6000,
        energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial)
        "Valve model, linear opening characteristics"
        annotation (Placement(transformation(extent={{-244,-26},{-224,-6}})));
      Modelica.Blocks.Sources.Ramp y(
        height=1,
        duration=1,
        offset=0) "Control signal"
        annotation (Placement(transformation(extent={{-298,68},{-278,88}})));
      CoSES_ProHMo.Storage.MyStratified tan1(
        redeclare package Medium = Buildings.Media.Water,
        m_flow_nominal=0.1,
        VTan(displayUnit="l") = 0.775,
        hTan=3,
        dIns=0.3,
        nSeg=10) annotation (Placement(transformation(extent={{-28,-314},{44,-242}})));
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
        annotation (Placement(transformation(extent={{-302,-260},{-242,-206}})));

      Test_prosumer1.Test_Pump_controler testClosedLoop_Pump annotation (
          Placement(transformation(extent={{-204,-360},{-184,-338}})));
      Modelica.Blocks.Sources.Pulse y1(
        offset=0.25,
        startTime=0,
        amplitude=0.5,
        period=15*60) "Input signal"
                     annotation (Placement(transformation(extent={{-248,-322},{
                -228,-302}})));
      Buildings.DHC.Loads.BaseClasses.BuildingTimeSeries buiHea(
        have_chiWat=false,
        filNam=
            "modelica://Buildings/Resources/Data/DHC/Loads/Examples/SwissResidential_20190916.mos",
        nPorts_aChiWat=1,
        nPorts_bChiWat=1,
        nPorts_aHeaWat=1,
        nPorts_bHeaWat=1)
        "Building with heating only"
        annotation (Placement(transformation(extent={{186,-60},{226,-4}})));

    equation
      connect(bat.SOC,con. SOC) annotation (Line(
          points={{407.5,-325.2},{360,-325.2},{360,-296},{370.75,-296}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(con.y,bat. P) annotation (Line(
          points={{392.625,-296},{435,-296},{435,-316}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(pv.terminal,conv. terminal_p) annotation (Line(
          points={{332,-318},{332,-320},{340,-320},{340,-420},{312,-420},{312,
              -453},{322,-453}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(bat.terminal,conv. terminal_p) annotation (Line(
          points={{460,-339},{472,-339},{472,-420},{312,-420},{312,-453},{322,
              -453}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(acLoad.terminal,gri. terminal) annotation (Line(
          points={{276,-330},{276,-392},{202,-392},{202,-380}},
          color={0,120,120},
          smooth=Smooth.None));
      connect(conv.terminal_n,gri. terminal) annotation (Line(
          points={{364,-453},{384,-453},{384,-512},{202,-512},{202,-380}},
          color={0,120,120},
          smooth=Smooth.None));
      connect(pv.weaBus, weaDat.weaBus) annotation (Line(
          points={{322,-309},{322,124},{228,124}},
          color={255,204,51},
          thickness=0.5));
      connect(testMyWolfCGB14_GC.ControlIn, realExpression.y) annotation (Line(
            points={{-399.226,6.3},{-413.665,6.3},{-413.665,16},{-421,16}},
                                                                color={0,0,127}));
      connect(testMyWolfCGB14_GC.term_p, gri.terminal) annotation (Line(points={{
              -371.263,-27},{-371.263,-392},{202,-392},{202,-380}},
                                                                color={0,120,120}));
      connect(tESTMyNeoTower2_GC.CHPOn, booleanExpression.y) annotation (Line(
            points={{-478.312,-90.1273},{-478.312,-112},{-517,-112}},
                                                                 color={255,0,255}));
      connect(tESTMyNeoTower2_GC.CHPModulation, realExpression1.y) annotation (
          Line(points={{-478.312,-110.527},{-504,-110.527},{-504,-156},{-517,
              -156}},
            color={0,0,127}));
      connect(tESTMyNeoTower2_GC.term_p, gri.terminal) annotation (Line(points={{-448.35,
              -149.782},{-352,-149.782},{-352,-392},{202,-392},{202,-380}},
            color={0,120,120}));
      connect(testMyWolfCGB14_GC.port_a, valLin1.port_1) annotation (Line(points={{
              -346.253,7.2},{-346.253,-16},{-244,-16}},   color={0,127,255}));
      connect(tESTMyNeoTower2_GC.port_a, valLin1.port_3) annotation (Line(points={{-434.75,
              -137.418},{-234,-137.418},{-234,-26}},        color={0,127,255}));
      connect(tan.port_a, valLin1.port_2) annotation (Line(points={{-108,36},{
              -108,48},{-212,48},{-212,-16},{-224,-16}},  color={0,127,255}));
      connect(valLin.port_3, tESTMyNeoTower2_GC.port_b) annotation (Line(points={{-162,
              -148},{-164,-148},{-164,-156},{-404,-156},{-404,-118.255},{-435.6,
              -118.255}},        color={0,127,255}));
      connect(valLin.port_1, testMyWolfCGB14_GC.port_b) annotation (Line(points={{-172,
              -138},{-172,-140},{-314,-140},{-314,-106},{-350.768,-106},{
              -350.768,31.8}}, color={0,127,255}));
      connect(y.y, valLin1.y) annotation (Line(points={{-277,78},{-234,78},{-234,
              -4}}, color={0,0,127}));
      connect(y.y, valLin.y) annotation (Line(points={{-277,78},{-236,78},{-236,8},
              {-162,8},{-162,-126}}, color={0,0,127}));
      connect(testMyWolfCGB14_GC.weaBus, weaDat.weaBus) annotation (Line(
          points={{-379.6,42.6},{-379.6,124},{228,124}},
          color={255,204,51},
          thickness=0.5));
      connect(tESTMyNeoTower2_GC.weaBus, weaDat.weaBus) annotation (Line(
          points={{-447.925,-72.5091},{-447.925,124},{228,124}},
          color={255,204,51},
          thickness=0.5));
      connect(solCol.port_a, testClosedLoop_Pump.port_a) annotation (Line(points=
              {{-302,-233},{-316,-233},{-316,-348.78},{-203.6,-348.78}}, color={0,
              127,255}));
      connect(y1.y, testClosedLoop_Pump.SetInSignal) annotation (Line(points={{
              -227,-312},{-202.2,-312},{-202.2,-336.9}}, color={0,0,127}));
      connect(tan1.port_b, testClosedLoop_Pump.port_b) annotation (Line(points={{
              8,-314},{8,-348.78},{-183.6,-348.78}}, color={0,127,255}));
      connect(solCol.port_b, tan1.port_a) annotation (Line(points={{-242,-233},{8,
              -233},{8,-242}}, color={0,127,255}));
      connect(solCol.weaBus, weaDat.weaBus) annotation (Line(
          points={{-302,-207.08},{-328,-207.08},{-328,124},{228,124}},
          color={255,204,51},
          thickness=0.5));
      connect(valLin.port_2, tan.fluPorVol[2]) annotation (Line(points={{-152,
              -138},{-152,-140},{-144,-140},{-144,-48},{-156,-48},{-156,-5.04},{
              -126,-5.04}}, color={0,127,255}));
      connect(tan.port_b, buiHea.ports_aHeaWat[1]) annotation (Line(points={{-108,
              -36},{-108,-48},{160,-48},{160,-37.6},{186,-37.6}}, color={0,127,
              255}));
      connect(buiHea.ports_bHeaWat[1], tan.fluPorVol[4]) annotation (Line(points={{226,
              -37.6},{240,-37.6},{240,-80},{-144,-80},{-144,-48},{-156,-48},{
              -156,-2.16},{-126,-2.16}},  color={0,127,255}));
      connect(buiHea.PPum, acLoad.Pow) annotation (Line(points={{227.333,-26.4},
              {276,-26.4},{276,-286}},
                                  color={0,0,127}));
      connect(testClosedLoop_Pump.term_p, gri.terminal) annotation (Line(points={
              {-188.4,-363.74},{-188.4,-392},{202,-392},{202,-380}}, color={0,120,
              120}));
      annotation (
        Icon(
          coordinateSystem(
            preserveAspectRatio=false, extent={{-400,-520},{440,120}})),
        Diagram(
            coordinateSystem(
            preserveAspectRatio=false, extent={{-400,-520},{440,120}})),
        __Dymola_Commands(
          file="modelica://Buildings/Resources/Scripts/Dymola/DHC/Loads/Heating/Examples/BuildingTimeSeriesWithETS.mos" "Simulate and plot"),
        experiment(
          StartTime=2592000,
          StopTime=3628800,
          Tolerance=1e-06),
        Documentation(info="<html>
<p>
This model provides an example for a building with loads provided
as time series and connected to a direct ETS for heating with the
return heating water temperature controlled below a maximum threshold.
</p>
</html>",     revisions="<html>
<ul>
<li>
March 20, 2022, by Chengnan Shi:<br/>
First implementation.
</li>
</ul>
</html>"));
    end HeatingBuildingTimeSeries_WolfCGB14_NeoTower_GC_Solar;

    model HeatingBuildingTimeSeries_MyWolfCHA10_GC
      "Example model of a building with loads provided as time series"
      extends Modelica.Icons.Example;
      package Medium=Buildings.Media.Water
        "Medium model";
      Buildings.Electrical.DC.Sources.PVSimpleOriented
                                             pv(
        A=200e3/800/0.12,
        til=0.34906585039887,
        azi=-0.78539816339745,
        V_nominal=480) "PV array"
        annotation (Placement(transformation(extent={{144,-154},{124,-134}})));
      Buildings.Electrical.DC.Storage.Battery bat(EMax=500e3*4*3600, V_nominal=
            480)       "Battery"
        annotation (Placement(transformation(extent={{272,-188},{222,-142}})));
      Buildings.Electrical.AC.OnePhase.Conversion.ACDCConverter conv(
          conversionFactor=480/480, eta=0.9)
                 "AC/DC converter"
        annotation (Placement(transformation(extent={{176,-302},{134,-256}})));
      Buildings.Electrical.AC.OnePhase.Sources.Grid gri(
        f=60,
        V=480,
        phiSou=0) annotation (Placement(transformation(extent={{-6,-206},{34,-166}})));
      Buildings.Examples.ChillerPlant.BaseClasses.Controls.BatteryControl
                                          con "Battery controller"
        annotation (Placement(transformation(extent={{184,-132},{204,-112}})));
      Buildings.Electrical.AC.OnePhase.Loads.Inductive acLoad(mode=Buildings.Electrical.Types.Load.VariableZ_P_input,
          V_nominal=480)
        annotation (Placement(transformation(extent={{22,-20},{-22,20}},
            rotation=270,
            origin={88,-134})));
      CoSES_ProHMo.Storage.MyStratified tan(
        redeclare package Medium = Buildings.Media.Water,
        m_flow_nominal=0.1,
        VTan(displayUnit="l") = 0.775,
        hTan=3,
        dIns=0.3,
        nSeg=10) annotation (Placement(transformation(extent={{-174,-28},{-102,44}})));
      Buildings.BoundaryConditions.WeatherData.ReaderTMY3 weaDat(filNam=
            ModelicaServices.ExternalReferences.loadResource(
            "modelica://Buildings/Resources/weatherdata/USA_CA_San.Francisco.Intl.AP.724940_TMY3.mos"))
        annotation (Placement(transformation(extent={{156,90},{176,110}})));
      Modelica.Blocks.Sources.RealExpression realExpression1(y=1)
        annotation (Placement(transformation(extent={{-544,-10},{-524,10}})));
      Modelica.Blocks.Sources.RealExpression realExpression5(y=1)
        annotation (Placement(transformation(extent={{-544,-42},{-524,-22}})));
      Modelica.Blocks.Sources.BooleanExpression booleanExpression(y=true)
        annotation (Placement(transformation(extent={{-542,30},{-522,50}})));
      Modelica.Blocks.Sources.BooleanExpression booleanExpression1(y=true)
        annotation (Placement(transformation(extent={{-540,60},{-520,80}})));
      Test_prosumer1.Test_MyWolfCHA10_GC test1_MyWolfCHA10_GC
        annotation (Placement(transformation(extent={{-452,20},{-412,60}})));
      Buildings.DHC.Loads.BaseClasses.BuildingTimeSeries buiHea(
        have_chiWat=false,
        filNam=
            "modelica://Buildings/Resources/Data/DHC/Loads/Examples/SwissResidential_20190916.mos",
        nPorts_aChiWat=1,
        nPorts_bChiWat=1,
        nPorts_aHeaWat=1,
        nPorts_bHeaWat=1)
        "Building with heating only"
        annotation (Placement(transformation(extent={{14,-36},{52,22}})));

    equation
      connect(bat.SOC,con. SOC) annotation (Line(
          points={{219.5,-151.2},{176,-151.2},{176,-122},{182.75,-122}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(con.y,bat. P) annotation (Line(
          points={{204.625,-122},{247,-122},{247,-142}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(pv.terminal,conv. terminal_p) annotation (Line(
          points={{144,-144},{134,-144},{134,-279}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(bat.terminal,conv. terminal_p) annotation (Line(
          points={{272,-165},{284,-165},{284,-218},{134,-218},{134,-279}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(acLoad.terminal,gri. terminal) annotation (Line(
          points={{88,-156},{88,-206},{14,-206}},
          color={0,120,120},
          smooth=Smooth.None));
      connect(conv.terminal_n,gri. terminal) annotation (Line(
          points={{176,-279},{188,-279},{188,-316},{14,-316},{14,-206}},
          color={0,120,120},
          smooth=Smooth.None));
      connect(pv.weaBus, weaDat.weaBus) annotation (Line(
          points={{134,-135},{134,100},{176,100}},
          color={255,204,51},
          thickness=0.5));
      connect(test1_MyWolfCHA10_GC.HPOn, booleanExpression1.y) annotation (Line(
            points={{-449.8,50.6},{-508,50.6},{-508,70},{-519,70}}, color={255,0,
              255}));
      connect(booleanExpression.y, test1_MyWolfCHA10_GC.HPMode) annotation (Line(
            points={{-521,40},{-464,40},{-464,43},{-449.8,43}}, color={255,0,255}));
      connect(test1_MyWolfCHA10_GC.HPModulation, realExpression1.y) annotation (
          Line(points={{-449.8,35},{-512,35},{-512,0},{-523,0}}, color={0,0,127}));
      connect(test1_MyWolfCHA10_GC.HPAuxModulation, realExpression5.y)
        annotation (Line(points={{-449.8,27.4},{-508,27.4},{-508,-32},{-523,-32}},
            color={0,0,127}));
      connect(tan.port_a, test1_MyWolfCHA10_GC.port_a) annotation (Line(points={{
              -138,44},{-140,44},{-140,56},{-388,56},{-388,38.8},{-404.4,38.8}},
            color={0,127,255}));
      connect(test1_MyWolfCHA10_GC.port_b, tan.fluPorVol[2]) annotation (Line(
            points={{-401.2,47.6},{-304,47.6},{-304,48},{-208,48},{-208,-2},{-158,
              -2},{-158,2.96},{-156,2.96}},                             color={0,
              127,255}));
      connect(test1_MyWolfCHA10_GC.weaBus, weaDat.weaBus) annotation (Line(
          points={{-420.4,63.2},{-420.4,100},{176,100}},
          color={255,204,51},
          thickness=0.5));
      connect(gri.terminal, test1_MyWolfCHA10_GC.term_p) annotation (Line(points=
              {{14,-206},{14,-220},{-420.8,-220},{-420.8,13.2}}, color={0,120,120}));
      connect(tan.port_b, buiHea.ports_aHeaWat[1]) annotation (Line(points={{-138,
              -28},{-140,-28},{-140,-40},{-28,-40},{-28,-12.8},{14,-12.8}},
                                                             color={0,127,255}));
      connect(buiHea.ports_bHeaWat[1], tan.fluPorVol[4]) annotation (Line(points={{52,
              -12.8},{64,-12.8},{64,-56},{-188,-56},{-188,5.84},{-156,5.84}},
            color={0,127,255}));
      connect(buiHea.PPum, acLoad.Pow) annotation (Line(points={{53.2667,-1.2},
              {88,-1.2},{88,-112}},
                                color={0,0,127}));
      annotation (
        Icon(
          coordinateSystem(
            preserveAspectRatio=false, extent={{-480,-320},{280,120}})),
        Diagram(
            coordinateSystem(
            preserveAspectRatio=false, extent={{-480,-320},{280,120}})),
        __Dymola_Commands(
          file="modelica://Buildings/Resources/Scripts/Dymola/DHC/Loads/Heating/Examples/BuildingTimeSeriesWithETS.mos" "Simulate and plot"),
        experiment(
          StartTime=2592000,
          StopTime=3628800,
          Tolerance=1e-06),
        Documentation(info="<html>
<p>
This model provides an example for a building with loads provided
as time series and connected to a direct ETS for heating with the
return heating water temperature controlled below a maximum threshold.
</p>
</html>",     revisions="<html>
<ul>
<li>
March 20, 2022, by Chengnan Shi:<br/>
First implementation.
</li>
</ul>
</html>"));
    end HeatingBuildingTimeSeries_MyWolfCHA10_GC;

    model CoolingBuildingTimeSeries_MyWolfCHA10_GC
      "Example model of a building with loads provided as time series"
      extends Modelica.Icons.Example;
      package Medium=Buildings.Media.Water
        "Medium model";
      Buildings.Electrical.DC.Sources.PVSimpleOriented
                                             pv(
        A=200e3/800/0.12,
        til=0.34906585039887,
        azi=-0.78539816339745,
        V_nominal=480) "PV array"
        annotation (Placement(transformation(extent={{144,-154},{124,-134}})));
      Buildings.Electrical.DC.Storage.Battery bat(EMax=500e3*4*3600, V_nominal=
            480)       "Battery"
        annotation (Placement(transformation(extent={{272,-188},{222,-142}})));
      Buildings.Electrical.AC.OnePhase.Conversion.ACDCConverter conv(
          conversionFactor=480/480, eta=0.9)
                 "AC/DC converter"
        annotation (Placement(transformation(extent={{176,-302},{134,-256}})));
      Buildings.Electrical.AC.OnePhase.Sources.Grid gri(
        f=60,
        V=480,
        phiSou=0) annotation (Placement(transformation(extent={{-6,-206},{34,-166}})));
      Buildings.Examples.ChillerPlant.BaseClasses.Controls.BatteryControl
                                          con "Battery controller"
        annotation (Placement(transformation(extent={{184,-132},{204,-112}})));
      Buildings.Electrical.AC.OnePhase.Loads.Inductive acLoad(mode=Buildings.Electrical.Types.Load.VariableZ_P_input,
          V_nominal=480)
        annotation (Placement(transformation(extent={{22,-20},{-22,20}},
            rotation=270,
            origin={88,-134})));
      CoSES_ProHMo.Storage.MyStratified tan(
        redeclare package Medium = Buildings.Media.Water,
        m_flow_nominal=0.1,
        VTan(displayUnit="l") = 0.775,
        hTan=3,
        dIns=0.3,
        nSeg=10) annotation (Placement(transformation(extent={{-174,-28},{-102,44}})));
      Buildings.BoundaryConditions.WeatherData.ReaderTMY3 weaDat(filNam=
            ModelicaServices.ExternalReferences.loadResource(
            "modelica://Buildings/Resources/weatherdata/USA_CA_San.Francisco.Intl.AP.724940_TMY3.mos"))
        annotation (Placement(transformation(extent={{156,90},{176,110}})));
      Modelica.Blocks.Sources.RealExpression realExpression1(y=1)
        annotation (Placement(transformation(extent={{-544,-10},{-524,10}})));
      Modelica.Blocks.Sources.RealExpression realExpression5(y=1)
        annotation (Placement(transformation(extent={{-544,-42},{-524,-22}})));
      Modelica.Blocks.Sources.BooleanExpression booleanExpression(y=false)
        annotation (Placement(transformation(extent={{-542,30},{-522,50}})));
      Modelica.Blocks.Sources.BooleanExpression booleanExpression1(y=true)
        annotation (Placement(transformation(extent={{-540,60},{-520,80}})));
      Test_prosumer1.Test_MyWolfCHA10_GC test1_MyWolfCHA10_GC
        annotation (Placement(transformation(extent={{-452,20},{-412,60}})));
      Buildings.DHC.Loads.BaseClasses.BuildingTimeSeries buiCoo(
        have_heaWat=false,
        filNam=
            "modelica://Buildings/Resources/Data/DHC/Loads/Examples/SwissResidential_20190916.mos",
        nPorts_aChiWat=1,
        nPorts_bChiWat=1)
        "Building wint cooling only"
        annotation (Placement(transformation(extent={{-2,-30},{30,18}})));

    equation
      connect(bat.SOC,con. SOC) annotation (Line(
          points={{219.5,-151.2},{176,-151.2},{176,-122},{182.75,-122}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(con.y,bat. P) annotation (Line(
          points={{204.625,-122},{247,-122},{247,-142}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(pv.terminal,conv. terminal_p) annotation (Line(
          points={{144,-144},{134,-144},{134,-279}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(bat.terminal,conv. terminal_p) annotation (Line(
          points={{272,-165},{284,-165},{284,-218},{134,-218},{134,-279}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(acLoad.terminal,gri. terminal) annotation (Line(
          points={{88,-156},{88,-206},{14,-206}},
          color={0,120,120},
          smooth=Smooth.None));
      connect(conv.terminal_n,gri. terminal) annotation (Line(
          points={{176,-279},{188,-279},{188,-316},{14,-316},{14,-206}},
          color={0,120,120},
          smooth=Smooth.None));
      connect(pv.weaBus, weaDat.weaBus) annotation (Line(
          points={{134,-135},{134,100},{176,100}},
          color={255,204,51},
          thickness=0.5));
      connect(test1_MyWolfCHA10_GC.HPOn, booleanExpression1.y) annotation (Line(
            points={{-449.8,50.6},{-508,50.6},{-508,70},{-519,70}}, color={255,0,
              255}));
      connect(booleanExpression.y, test1_MyWolfCHA10_GC.HPMode) annotation (Line(
            points={{-521,40},{-464,40},{-464,43},{-449.8,43}}, color={255,0,255}));
      connect(test1_MyWolfCHA10_GC.HPModulation, realExpression1.y) annotation (
          Line(points={{-449.8,35},{-512,35},{-512,0},{-523,0}}, color={0,0,127}));
      connect(test1_MyWolfCHA10_GC.HPAuxModulation, realExpression5.y)
        annotation (Line(points={{-449.8,27.4},{-508,27.4},{-508,-32},{-523,-32}},
            color={0,0,127}));
      connect(tan.port_a, test1_MyWolfCHA10_GC.port_a) annotation (Line(points={{
              -138,44},{-140,44},{-140,56},{-388,56},{-388,38.8},{-404.4,38.8}},
            color={0,127,255}));
      connect(test1_MyWolfCHA10_GC.weaBus, weaDat.weaBus) annotation (Line(
          points={{-420.4,63.2},{-420.4,100},{176,100}},
          color={255,204,51},
          thickness=0.5));
      connect(gri.terminal, test1_MyWolfCHA10_GC.term_p) annotation (Line(points=
              {{14,-206},{14,-220},{-420.8,-220},{-420.8,13.2}}, color={0,120,120}));
      connect(tan.port_b, buiCoo.ports_aChiWat[1]) annotation (Line(points={{-138,
              -28},{-140,-28},{-140,-40},{-36,-40},{-36,-20.4},{-2,-20.4}},
                                                                         color={0,
              127,255}));
      connect(buiCoo.ports_bChiWat[1], tan.fluPorVol[4]) annotation (Line(points={{30,
              -20.4},{30,-52},{-188,-52},{-188,5.84},{-156,5.84}},
                      color={0,127,255}));
      connect(test1_MyWolfCHA10_GC.port_b, tan.fluPorVol[2]) annotation (Line(
            points={{-401.2,47.6},{-188,47.6},{-188,2.96},{-156,2.96}}, color={0,
              127,255}));
      connect(buiCoo.PPum, acLoad.Pow) annotation (Line(points={{31.0667,-1.2},
              {88,-1.2},{88,-112}},   color={0,0,127}));
      annotation (
        Icon(
          coordinateSystem(
            preserveAspectRatio=false, extent={{-480,-320},{280,120}})),
        Diagram(
            coordinateSystem(
            preserveAspectRatio=false, extent={{-480,-320},{280,120}})),
        __Dymola_Commands(
          file="modelica://Buildings/Resources/Scripts/Dymola/DHC/Loads/Heating/Examples/BuildingTimeSeriesWithETS.mos" "Simulate and plot"),
        experiment(
          StartTime=2592000,
          StopTime=3628800,
          Tolerance=1e-06),
        Documentation(info="<html>
<p>
This model provides an example for a building with loads provided
as time series and connected to a direct ETS for heating with the
return heating water temperature controlled below a maximum threshold.
</p>
</html>",     revisions="<html>
<ul>
<li>
March 20, 2022, by Chengnan Shi:<br/>
First implementation.
</li>
</ul>
</html>"));
    end CoolingBuildingTimeSeries_MyWolfCHA10_GC;
  end Experiments;
end Houses;
