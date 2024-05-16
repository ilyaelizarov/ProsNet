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
    CoSES_ProHMo.Storage.MyStratified tan(
      redeclare package Medium = Buildings.Media.Water,
      m_flow_nominal=0.1,
      VTan(displayUnit="l") = 0.775,
      hTan=3,
      dIns=0.3,
      nSeg=10) annotation (Placement(transformation(extent={{-48,-62},{60,46}})));
    Buildings.BoundaryConditions.WeatherData.ReaderTMY3 weaDat(filNam=
          ModelicaServices.ExternalReferences.loadResource(
          "modelica://Buildings/Resources/weatherdata/USA_CA_San.Francisco.Intl.AP.724940_TMY3.mos"))
      annotation (Placement(transformation(extent={{210,110},{230,130}})));
    Modelica.Blocks.Sources.RealExpression realExpression(y=1)
      annotation (Placement(transformation(extent={{-532,6},{-512,26}})));
    Test_prosumer1.TestMyWolfCGB14_GC testMyWolfCGB14_GC
      annotation (Placement(transformation(extent={{-440,-18},{-302,62}})));
    Test_prosumer1.TESTMyNeoTower2_GC tESTMyNeoTower2_GC
      annotation (Placement(transformation(extent={{-520,-164},{-422,-96}})));
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

    Test_prosumer1.Test_Pump_controler testClosedLoop_Pump(m_flow_nominal=0.5)
      annotation (Placement(transformation(extent={{-16,-264},{4,-242}})));
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
      nPorts_aHeaWat=2,
      nPorts_bHeaWat=2)
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
    connect(testMyWolfCGB14_GC.ControlIn, realExpression.y) annotation (Line(
          points={{-425.837,11.4545},{-425.837,16},{-511,16}},color={0,0,127}));
    connect(testMyWolfCGB14_GC.term_p, gri.terminal) annotation (Line(points={{
            -367.368,-28.9091},{-367.368,-410},{82,-410}},    color={0,120,120}));
    connect(tESTMyNeoTower2_GC.CHPOn, booleanExpression.y) annotation (Line(
          points={{-503.156,-90.1273},{-503.156,-88},{-541,-88}},
                                                               color={255,0,255}));
    connect(tESTMyNeoTower2_GC.CHPModulation, realExpression1.y) annotation (
        Line(points={{-503.156,-110.527},{-503.156,-110},{-537,-110}},
          color={0,0,127}));
    connect(tESTMyNeoTower2_GC.term_p, gri.terminal) annotation (Line(points={{
            -459.975,-149.782},{-459.975,-410},{82,-410}},
          color={0,120,120}));
    connect(testMyWolfCGB14_GC.port_a, valLin1.port_1) annotation (Line(points={{
            -315.074,12.5455},{-256,12.5455},{-256,-16},{-244,-16}},
                                                        color={0,127,255}));
    connect(tESTMyNeoTower2_GC.port_a, valLin1.port_3) annotation (Line(points={{
            -440.375,-137.418},{-234,-137.418},{-234,-26}},
                                                          color={0,127,255}));
    connect(valLin.port_3, tESTMyNeoTower2_GC.port_b) annotation (Line(points={{-162,
            -148},{-164,-148},{-164,-178},{-386,-178},{-386,-118},{-414,-118},{
            -414,-118.255},{-441.6,-118.255}},
                               color={0,127,255}));
    connect(valLin.port_1, testMyWolfCGB14_GC.port_b) annotation (Line(points={{-172,
            -138},{-172,-140},{-184,-140},{-184,100},{-324.516,100},{-324.516,
            42.3636}},       color={0,127,255}));
    connect(y.y, valLin1.y) annotation (Line(points={{-269,66},{-234,66},{-234,
            -4}}, color={0,0,127}));
    connect(y.y, valLin.y) annotation (Line(points={{-269,66},{-162,66},{-162,
            -126}},                color={0,0,127}));
    connect(testMyWolfCGB14_GC.weaBus, weaDat.weaBus) annotation (Line(
        points={{-384.8,55.4545},{-384.8,120},{230,120}},
        color={255,204,51},
        thickness=0.5));
    connect(tESTMyNeoTower2_GC.weaBus, weaDat.weaBus) annotation (Line(
        points={{-459.363,-72.5091},{-459.363,120},{230,120}},
        color={255,204,51},
        thickness=0.5));
    connect(solCol.port_a, testClosedLoop_Pump.port_a) annotation (Line(points={{-334,
            -289},{-340,-289},{-340,-252.78},{-15.6,-252.78}},         color={0,
            127,255}));
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
    connect(testClosedLoop_Pump.port_b, DHWTan.port_bHea) annotation (Line(
          points={{4.4,-252.78},{124,-252.78},{124,-273.2},{160,-273.2}}, color
          ={0,127,255}));
    connect(solCol.port_b, DHWTan.port_aHea) annotation (Line(points={{-274,
            -289},{-274,-300},{244,-300},{244,-273.2},{222,-273.2}}, color={0,
            127,255}));
    connect(booToRea.y, testClosedLoop_Pump.SetInSignal) annotation (Line(
          points={{10.4,-218},{10.4,-220},{-14.2,-220},{-14.2,-240.9}}, color={
            0,0,127}));
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
    connect(valLin1.port_2, tan.port_a) annotation (Line(points={{-224,-16},{
            -108,-16},{-108,68},{6,68},{6,46}}, color={0,127,255}));
    connect(testClosedLoop_Pump.term_p, gri.terminal) annotation (Line(points={
            {-0.4,-267.74},{-0.4,-410},{82,-410}}, color={0,120,120}));
    connect(theMixVal.port_col, souCol.ports[1]) annotation (Line(points={{254,
            -215.2},{236,-215.2},{236,-188},{216,-188}}, color={0,127,255}));
    connect(tan.port_b, buiHea.ports_aHeaWat[1]) annotation (Line(points={{6,-62},
            {6,-80},{164,-80},{164,-59.4667},{174,-59.4667}},color={0,127,255}));
    connect(buiHea.PPum, acLoad.Pow) annotation (Line(points={{250.467,-45.6},{
            290,-45.6},{290,-46},{300,-46},{300,-328}},
                                color={0,0,127}));
    connect(valLin.port_2, tan.fluPorVol[2]) annotation (Line(points={{-152,
            -138},{-152,-140},{-26,-140},{-26,-22},{-21,-22},{-21,-15.56}},
          color={0,127,255}));
    connect(buiHea.ports_bHeaWat[1], valLin2.port_1) annotation (Line(points={{248,
            -59.4667},{260,-59.4667},{260,-124},{130,-124}},
                                                           color={0,127,255}));
    connect(DHWTan.port_aDom, valLin2.port_3) annotation (Line(points={{160,
            -234.8},{104,-234.8},{104,-114},{120,-114}}, color={0,127,255}));
    connect(valLin2.port_2, tan.fluPorVol[4]) annotation (Line(points={{110,
            -124},{-40,-124},{-40,-12},{-21,-12},{-21,-11.24}},
                                            color={0,127,255}));
    connect(y1.y, valLin2.y) annotation (Line(points={{119,20},{140,20},{140,
            -148},{120,-148},{120,-136}}, color={0,0,127}));
    connect(buiHea.ports_aHeaWat[2], sinZonFlo.ports[1]) annotation (Line(
          points={{174,-57.3333},{174,-66},{204.15,-66}},
          color={0,127,255}));
    connect(buiHea.ports_bHeaWat[2], sinZonFlo.ports[2]) annotation (Line(
          points={{248,-57.3333},{260,-57.3333},{260,-124},{172,-124},{172,-66},
            {204.65,-66}},                                               color=
            {0,127,255}));
    connect(sinZonFlo.weaBus, weaDat.weaBus) annotation (Line(
        points={{203.4,-51.5},{164,-51.5},{164,120},{230,120}},
        color={255,204,51},
        thickness=0.5));
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
    CoSES_ProHMo.Storage.MyStratified tan(
      redeclare package Medium = Buildings.Media.Water,
      m_flow_nominal=0.1,
      VTan(displayUnit="l") = 0.775,
      hTan=3,
      dIns=0.3,
      nSeg=10) annotation (Placement(transformation(extent={{-48,-62},{60,46}})));
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

    Test_prosumer1.Test_Pump_controler testClosedLoop_Pump(m_flow_nominal=0.5)
      annotation (Placement(transformation(extent={{-178,-144},{-158,-122}})));
    Test_prosumer1.Test_MyWolfCGB20_GC test_MyWolfCGB20_GC
      annotation (Placement(transformation(extent={{-432,-46},{-372,14}})));
    Test_prosumer1.Test_MyWolfCHA10_GC test_MyWolfCHA10_GC
      annotation (Placement(transformation(extent={{-420,-330},{-344,-254}})));
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
    connect(realExpression1.y, testClosedLoop_Pump.SetInSignal) annotation (
        Line(points={{-189,-118},{-189,-120.9},{-176.2,-120.9}}, color={0,0,127}));
    connect(solCol.weaBus, weaDat.weaBus) annotation (Line(
        points={{-338,-107.08},{-360,-107.08},{-360,120},{230,120}},
        color={255,204,51},
        thickness=0.5));
    connect(solCol.port_b, testClosedLoop_Pump.port_a) annotation (Line(points={{-278,
            -133},{-232.8,-133},{-232.8,-132.78},{-177.6,-132.78}},
          color={0,127,255}));
    connect(valLin2.port_2, tan.port_a) annotation (Line(points={{-166,0},{-96,
            0},{-96,64},{6,64},{6,46}}, color={0,127,255}));
    connect(y.y, valLin1.y) annotation (Line(points={{-271,66},{-176,66},{-176,
            64},{-120,64},{-120,-84}}, color={0,0,127}));
    connect(testClosedLoop_Pump.term_p, gri.terminal) annotation (Line(points={{-162.4,
            -147.74},{-162.4,-410},{82,-410}},
                                 color={0,120,120}));
    connect(test_MyWolfCGB20_GC.ControlIn, realExpression.y) annotation (Line(
          points={{-435.3,-15.1},{-464.15,-15.1},{-464.15,-14},{-493,-14}},
          color={0,0,127}));
    connect(test_MyWolfCGB20_GC.term_p, gri.terminal) annotation (Line(points={{-384,
            -49},{-388,-49},{-388,-410},{82,-410}},       color={0,120,120}));
    connect(test_MyWolfCGB20_GC.port_a, valLin2.port_1) annotation (Line(points={{-330,
            -22.6},{-330,0},{-186,0}},       color={0,127,255}));
    connect(test_MyWolfCGB20_GC.weaBus, weaDat.weaBus) annotation (Line(
        points={{-398.4,20.6},{-398.4,120},{230,120}},
        color={255,204,51},
        thickness=0.5));
    connect(booleanExpression1.y, test_MyWolfCHA10_GC.HPOn) annotation (Line(
          points={{-467,-250},{-467,-252},{-432,-252},{-432,-271.86},{-415.82,
            -271.86}},
          color={255,0,255}));
    connect(test_MyWolfCHA10_GC.HPMode, booleanExpression.y) annotation (Line(
          points={{-415.82,-286.3},{-456,-286.3},{-456,-270},{-465,-270}},
          color={255,0,255}));
    connect(test_MyWolfCHA10_GC.HPModulation, realExpression2.y) annotation (
        Line(points={{-415.82,-301.5},{-441.41,-301.5},{-441.41,-300},{-467,
            -300}},
          color={0,0,127}));
    connect(test_MyWolfCHA10_GC.HPAuxModulation, realExpression5.y) annotation (
       Line(points={{-415.82,-315.94},{-420,-315.94},{-420,-320},{-469,-320}},
          color={0,0,127}));
    connect(test_MyWolfCHA10_GC.weaBus, weaDat.weaBus) annotation (Line(
        points={{-359.96,-247.92},{-359.96,120},{230,120}},
        color={255,204,51},
        thickness=0.5));
    connect(test_MyWolfCHA10_GC.term_p, gri.terminal) annotation (Line(points={{-360.72,
            -342.92},{-360.72,-410},{82,-410}},          color={0,120,120}));
    connect(test_MyWolfCHA10_GC.port_b, valLin3.port_1) annotation (Line(points={{-323.48,
            -277.56},{-320,-277.56},{-320,-274},{-210,-274}},
          color={0,127,255}));
    connect(test_MyWolfCHA10_GC.port_a, valLin4.port_1) annotation (Line(points={{-329.56,
            -294.28},{-329.56,-310},{-136,-310}},          color={0,127,255}));
    connect(valLin6.y, y1.y)
      annotation (Line(points={{164,-26},{164,88},{117,88}}, color={0,0,127}));
    connect(valLin6.port_1, tan.port_b) annotation (Line(points={{154,-38},{80,
            -38},{80,-62},{6,-62}}, color={0,127,255}));
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
            -47.2},{304,-47.2},{304,-100},{40,-100},{40,-88},{8,-88},{8,-114},{
            14,-114}},                                                color={0,
            127,255}));
    connect(domHotWat.port_aHea, valLin5.port_3) annotation (Line(points={{208,
            -289.6},{220,-289.6},{220,-136},{24,-136},{24,-124}}, color={0,127,
            255}));
    connect(valLin5.port_2, tan.fluPorVol[4]) annotation (Line(points={{34,-114},
            {44,-114},{44,-76},{24,-76},{24,-80},{-60,-80},{-60,-10},{-20,-10},
            {-20,-11.24},{-21,-11.24}}, color={0,127,255}));
    connect(y1.y, valLin5.y) annotation (Line(points={{117,88},{164,88},{164,
            -16},{148,-16},{148,-92},{24,-92},{24,-102}}, color={0,0,127}));
    connect(test_MyWolfCGB20_GC.port_b, valLin1.port_1) annotation (Line(points
          ={{-345.6,-4.6},{-200,-4.6},{-200,-40},{-140,-40},{-140,-96},{-130,
            -96}}, color={0,127,255}));
    connect(valLin1.port_2, tan.fluPorVol[2]) annotation (Line(points={{-110,
            -96},{-70,-96},{-70,-24},{-22,-24},{-22,-15.56},{-21,-15.56}},
          color={0,127,255}));
    connect(solCol.port_a, valLin1.port_3) annotation (Line(points={{-338,-133},
            {-344,-133},{-344,-96},{-140,-96},{-140,-116},{-120,-116},{-120,
            -106}}, color={0,127,255}));
    connect(testClosedLoop_Pump.port_b, valLin2.port_3) annotation (Line(points
          ={{-157.6,-132.78},{-158,-132.78},{-158,-26},{-176,-26},{-176,-10}},
          color={0,127,255}));
    connect(valLin4.port_3, bui.ports_aChiWat[1]) annotation (Line(points={{-126,
            -320},{-128,-320},{-128,-328},{88,-328},{88,-96},{216,-96},{216,
            -62.8},{228,-62.8}},     color={0,127,255}));
    connect(valLin3.port_3, bui.ports_bChiWat[1]) annotation (Line(points={{-200,
            -284},{-200,-288},{32,-288},{32,-132},{312,-132},{312,-62.8},{298,
            -62.8}},     color={0,127,255}));
    connect(bui.ports_aHeaWat[1], valLin6.port_2) annotation (Line(points={{228,
            -47.2},{188,-47.2},{188,-38},{174,-38}}, color={0,127,255}));
    connect(valLin4.port_2, tan.port_a) annotation (Line(points={{-116,-310},{
            -116,-312},{-84,-312},{-84,50},{6,50},{6,46}}, color={0,127,255}));
    connect(valLin3.port_2, tan.fluPorVol[3]) annotation (Line(points={{-190,
            -274},{-72,-274},{-72,-16},{-24,-16},{-24,-13.4},{-21,-13.4}},
          color={0,127,255}));
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
    CoSES_ProHMo.Storage.MyStratified tan(
      redeclare package Medium = Buildings.Media.Water,
      m_flow_nominal=0.1,
      VTan(displayUnit="l") = 0.775,
      hTan=3,
      dIns=0.3,
      nSeg=10) annotation (Placement(transformation(extent={{-48,-62},{60,46}})));
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
    Test_prosumer1.Test_MyWolfBWS1_10_GC test_MyWolfBWS1_10_GC
      annotation (Placement(transformation(extent={{-414,-56},{-340,18}})));
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

    Test_prosumer1.Test_Pump_controler testClosedLoop_Pump(m_flow_nominal=0.5)
      annotation (Placement(transformation(extent={{-180,-308},{-160,-286}})));
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
    connect(uHea.y, test_MyWolfBWS1_10_GC.HPOn) annotation (Line(points={{
            -494.4,2},{-432,2},{-432,-3.09},{-412.89,-3.09}}, color={255,0,255}));
    connect(weaDat.weaBus, test_MyWolfBWS1_10_GC.weaBus) annotation (Line(
        points={{230,120},{-372.56,120},{-372.56,26.14}},
        color={255,204,51},
        thickness=0.5));
    connect(test_MyWolfBWS1_10_GC.term_p, gri.terminal) annotation (Line(points
          ={{-356.28,-68.58},{-356.28,-412},{52,-412},{52,-410},{82,-410}},
          color={0,120,120}));
    connect(realExpression.y, test_MyWolfBWS1_10_GC.HPAuxModulation)
      annotation (Line(points={{-511,-40},{-461.945,-40},{-461.945,-40.09},{
            -412.89,-40.09}}, color={0,0,127}));
    connect(realExpression1.y, testClosedLoop_Pump.SetInSignal) annotation (
        Line(points={{-195,-282},{-195,-284.9},{-178.2,-284.9}}, color={0,0,127}));
    connect(domHotWat.port_bHea, tan.port_b) annotation (Line(points={{144,
            -289.6},{6,-289.6},{6,-62}},           color={0,127,255}));
    connect(solCol.weaBus, weaDat.weaBus) annotation (Line(
        points={{-322,-271.08},{-322,120},{230,120}},
        color={255,204,51},
        thickness=0.5));
    connect(solCol.port_b, testClosedLoop_Pump.port_a) annotation (Line(points={{-262,
            -297},{-221.8,-297},{-221.8,-296.78},{-179.6,-296.78}},
          color={0,127,255}));
    connect(test_MyWolfBWS1_10_GC.port_a, valLin2.port_3) annotation (Line(
          points={{-301.52,-18.26},{-300,-18.26},{-300,-160},{-142,-160}},
                                                              color={0,127,255}));
    connect(valLin2.port_2, tan.port_a) annotation (Line(points={{-132,-150},{
            -124,-150},{-124,68},{6,68},{6,46}},
                                        color={0,127,255}));
    connect(solCol.port_a, valLin1.port_1) annotation (Line(points={{-322,-297},
            {-336,-297},{-336,-58},{-220,-58}},                         color={
            0,127,255}));
    connect(y.y, valLin1.y) annotation (Line(points={{-271,66},{-210,66},{-210,
            -46}},                     color={0,0,127}));
    connect(testClosedLoop_Pump.term_p, gri.terminal) annotation (Line(points={
            {-164.4,-311.74},{-164.4,-362},{-164,-362},{-164,-412},{52,-412},{
            52,-410},{82,-410}}, color={0,120,120}));
    connect(buiHea.ports_aHeaWat[1], tan.port_b) annotation (Line(points={{184,
            -11.1667},{184,-12},{72,-12},{72,-84},{6,-84},{6,-62}},
                                               color={0,127,255}));
    connect(buiHea.ports_bHeaWat[1], tan.fluPorVol[4]) annotation (Line(points={{258,
            -11.1667},{272,-11.1667},{272,-92},{-60,-92},{-60,-11.24},{-21,
            -11.24}},
          color={0,127,255}));
    connect(testClosedLoop_Pump.port_b, valLin2.port_3) annotation (Line(points
          ={{-159.6,-296.78},{-142,-296.78},{-142,-160}}, color={0,127,255}));
    connect(test_MyWolfBWS1_10_GC.port_b, valLin1.port_3) annotation (Line(
          points={{-303,4.68},{-288,4.68},{-288,-68},{-210,-68}}, color={0,127,
            255}));
    connect(valLin1.port_2, tan.fluPorVol[2]) annotation (Line(points={{-200,
            -58},{-200,-60},{-70,-60},{-70,-15.56},{-21,-15.56}}, color={0,127,
            255}));
    connect(domHotWat.port_aHea, tan.fluPorVol[6]) annotation (Line(points={{
            208,-289.6},{220,-289.6},{220,-316},{-80,-316},{-80,-2},{-22,-2},{
            -22,-6.92},{-21,-6.92}}, color={0,127,255}));
    connect(acLoad.Pow, buiHea.PPum) annotation (Line(points={{300,-328},{300,
            -228},{304,-228},{304,4},{260.467,4}}, color={0,0,127}));
    connect(sinZonFlo.ports[2], buiHea.ports_bHeaWat[2]) annotation (Line(
          points={{216.65,-10},{192,-10},{192,4},{172,4},{172,-60},{272,-60},{
            272,-8.83333},{258,-8.83333}},
                      color={0,127,255}));
    connect(sinZonFlo.ports[1], buiHea.ports_aHeaWat[2]) annotation (Line(
          points={{216.15,-10},{200,-10},{200,-8.83333},{184,-8.83333}},
                                     color={0,127,255}));
    connect(sinZonFlo.weaBus, weaDat.weaBus) annotation (Line(
        points={{215.4,4.5},{215.4,104},{240,104},{240,120},{230,120}},
        color={255,204,51},
        thickness=0.5));
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
    CoSES_ProHMo.Storage.MyStratified tan(
      redeclare package Medium = Buildings.Media.Water,
      m_flow_nominal=0.1,
      VTan(displayUnit="l") = 0.775,
      hTan=3,
      dIns=0.3,
      nSeg=10) annotation (Placement(transformation(extent={{-48,-62},{60,46}})));
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
    Test_prosumer1.Test_MyWolfCGB20_GC test_MyWolfCGB20_GC
      annotation (Placement(transformation(extent={{-432,-44},{-366,22}})));
    Buildings.DHC.Loads.BaseClasses.BuildingTimeSeries buiHea(
      have_chiWat=false,
      filNam=
          "modelica://Buildings/Resources/Data/DHC/Loads/Examples/SwissResidential_20190916.mos",
      nPorts_aChiWat=1,
      nPorts_bChiWat=1,
      nPorts_aHeaWat=2,
      nPorts_bHeaWat=2)
      "Building with heating only"
      annotation (Placement(transformation(extent={{210,-82},{284,-26}})));

    Buildings.ThermalZones.Detailed.Validation.BaseClasses.SingleZoneFloor sinZonFlo(
      redeclare package Medium =
          Modelica.Media.Water.ConstantPropertyLiquidWater,
      use_windPressure=false,
      gai(K=0*[0.4; 0.4; 0.2]))
      "Single-zone floor model"
      annotation (Placement(transformation(extent={{232,-64},{254,-42}})));
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
    connect(domHotWat.port_bHea, tan.port_b) annotation (Line(points={{144,
            -289.6},{6,-289.6},{6,-62}}, color={0,127,255}));
    connect(eleHea.port_a, tan.fluPorVol[3]) annotation (Line(points={{-264,
            -199},{-268,-199},{-268,-224},{-48,-224},{-48,-13.4},{-21,-13.4}},
          color={0,127,255}));
    connect(eleHea.port_b, tan.fluPorVol[8]) annotation (Line(points={{-216,
            -199},{-128,-199},{-128,6},{-22,6},{-22,-2.6},{-21,-2.6}}, color={0,
            127,255}));
    connect(test_MyWolfCGB20_GC.port_a, tan.port_a) annotation (Line(points={{
            -319.8,-18.26},{-319.8,-8},{-344,-8},{-344,68},{6,68},{6,46}},
          color={0,127,255}));
    connect(realExpression.y, test_MyWolfCGB20_GC.ControlIn) annotation (Line(
          points={{-511,16},{-452,16},{-452,-10.01},{-435.63,-10.01}}, color={0,
            0,127}));
    connect(test_MyWolfCGB20_GC.weaBus, weaDat.weaBus) annotation (Line(
        points={{-395.04,29.26},{-395.04,120},{230,120}},
        color={255,204,51},
        thickness=0.5));
    connect(test_MyWolfCGB20_GC.term_p, gri.terminal) annotation (Line(points={
            {-379.2,-47.3},{-379.2,-410},{82,-410}}, color={0,120,120}));
    connect(tan.port_b, buiHea.ports_aHeaWat[1]) annotation (Line(points={{6,-62},
            {6,-80},{200,-80},{200,-60.5333},{210,-60.5333}},
                                         color={0,127,255}));
    connect(domHotWat.port_aHea, tan.fluPorVol[6]) annotation (Line(points={{
            208,-289.6},{220,-289.6},{220,-316},{-62,-316},{-62,0},{-21,0},{-21,
            -6.92}}, color={0,127,255}));
    connect(buiHea.ports_bHeaWat[1], tan.fluPorVol[4]) annotation (Line(points={{284,
            -60.5333},{296,-60.5333},{296,-96},{-60,-96},{-60,-24},{-21,-24},{
            -21,-11.24}},
                      color={0,127,255}));
    connect(test_MyWolfCGB20_GC.port_b, tan.fluPorVol[2]) annotation (Line(
          points={{-336.96,1.54},{-336,1.54},{-336,-22},{-21,-22},{-21,-15.56}},
          color={0,127,255}));
    connect(buiHea.PPum, acLoad.Pow) annotation (Line(points={{286.467,-48.4},{
            304,-48.4},{304,-312},{318,-312},{318,-328}},
                                                    color={0,0,127}));
    connect(sinZonFlo.ports[2], buiHea.ports_bHeaWat[2]) annotation (Line(
          points={{237.115,-59.6},{263.42,-59.6},{263.42,-58.6667},{284,
            -58.6667}}, color={0,127,255}));
    connect(sinZonFlo.ports[1], buiHea.ports_aHeaWat[2]) annotation (Line(
          points={{236.565,-59.6},{236.565,-58.6667},{210,-58.6667}},
                        color={0,127,255}));
    connect(sinZonFlo.weaBus, weaDat.weaBus) annotation (Line(
        points={{235.74,-43.65},{235.74,-40},{238,-40},{238,104},{240,104},{240,
            120},{230,120}},
        color={255,204,51},
        thickness=0.5));
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
    CoSES_ProHMo.Storage.MyStratified tan(
      redeclare package Medium = Buildings.Media.Water,
      m_flow_nominal=0.1,
      VTan(displayUnit="l") = 0.775,
      hTan=3,
      dIns=0.3,
      nSeg=10) annotation (Placement(transformation(extent={{-48,-62},{60,46}})));
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
      annotation (Placement(transformation(extent={{-518,-242},{-498,-222}})));
    Test_prosumer1.Test_MyNeoTower5_GC test_MyNeoTower5_GC
      annotation (Placement(transformation(extent={{-396,-288},{-326,-218}})));
    Test_prosumer1.Test_MyWolfCGB50_GC test_MyWolfCGB50_GC
      annotation (Placement(transformation(extent={{-452,-10},{-394,48}})));
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
    connect(domHotWat.port_bHea, tan.port_b) annotation (Line(points={{144,
            -289.6},{6,-289.6},{6,-62}}, color={0,127,255}));
    connect(realExpression.y, test_MyWolfCGB50_GC.ControlIn) annotation (Line(
          points={{-511,16},{-484,16},{-484,19.87},{-454.61,19.87}}, color={0,0,
            127}));
    connect(y.y, valLin1.y) annotation (Line(points={{-289,94},{-266,94},{-266,
            -8}}, color={0,0,127}));
    connect(test_MyWolfCGB50_GC.port_a, valLin1.port_1) annotation (Line(points
          ={{-380.08,17.26},{-380.08,-20},{-276,-20}}, color={0,127,255}));
    connect(test_MyNeoTower5_GC.port_a, valLin1.port_3) annotation (Line(points={{-317.6,
            -258.6},{-266,-258.6},{-266,-30}},
          color={0,127,255}));
    connect(valLin1.port_2, tan.port_a) annotation (Line(points={{-256,-20},{
            -132,-20},{-132,68},{6,68},{6,46}}, color={0,127,255}));
    connect(uHea.y, test_MyNeoTower5_GC.CHPOn) annotation (Line(points={{-497,
            -232},{-408,-232},{-408,-235.15},{-389.35,-235.15}}, color={255,0,
            255}));
    connect(realExpression1.y, test_MyNeoTower5_GC.CHPModulation) annotation (
        Line(points={{-443,-258},{-443,-260},{-408,-260},{-408,-256.85},{
            -389.35,-256.85}}, color={0,0,127}));
    connect(test_MyNeoTower5_GC.port_b, valLin2.port_3) annotation (Line(points
          ={{-325.3,-237.6},{-214,-237.6},{-214,-88}}, color={0,127,255}));
    connect(test_MyWolfCGB50_GC.port_b, valLin2.port_1) annotation (Line(points
          ={{-387.62,38.72},{-232,38.72},{-232,-78},{-224,-78}}, color={0,127,
            255}));
    connect(weaDat.weaBus, test_MyWolfCGB50_GC.weaBus) annotation (Line(
        points={{230,120},{-419.52,120},{-419.52,54.38}},
        color={255,204,51},
        thickness=0.5));
    connect(test_MyNeoTower5_GC.weaBus, weaDat.weaBus) annotation (Line(
        points={{-340.7,-212.4},{-340.7,120},{230,120}},
        color={255,204,51},
        thickness=0.5));
    connect(gri.terminal, test_MyNeoTower5_GC.term_p) annotation (Line(points={
            {82,-410},{-342,-410},{-342,-354},{-341.4,-354},{-341.4,-299.9}},
          color={0,120,120}));
    connect(y.y, valLin2.y) annotation (Line(points={{-289,94},{-216,94},{-216,
            -66},{-214,-66}}, color={0,0,127}));
    connect(test_MyWolfCGB50_GC.term_p, gri.terminal) annotation (Line(points={
            {-405.6,-12.9},{-405.6,-410},{82,-410}}, color={0,120,120}));
    connect(valLin2.port_2, tan.fluPorVol[2]) annotation (Line(points={{-204,
            -78},{-48,-78},{-48,-20},{-21,-20},{-21,-15.56}}, color={0,127,255}));
    connect(domHotWat.port_aHea, tan.fluPorVol[6]) annotation (Line(points={{
            208,-289.6},{220,-289.6},{220,-316},{-62,-316},{-62,0},{-21,0},{-21,
            -6.92}}, color={0,127,255}));
    connect(buiHea.ports_bHeaWat[1], tan.fluPorVol[4]) annotation (Line(points={{244,
            -32.4},{256,-32.4},{256,-80},{-48,-80},{-48,-20},{-21,-20},{-21,
            -11.24}}, color={0,127,255}));
    connect(buiHea.ports_aHeaWat[1], tan.port_b) annotation (Line(points={{170,
            -32.4},{80,-32.4},{80,-84},{6,-84},{6,-62}},               color={0,
            127,255}));
    connect(buiHea.PPum, acLoad.Pow) annotation (Line(points={{246.467,-16.8},{
            264,-16.8},{264,-168},{320,-168},{320,-328}},
                                color={0,0,127}));
    connect(sinZonFlo.ports[1], buiHea.ports_aHeaWat[2]) annotation (Line(
          points={{198.98,-29.2},{184,-29.2},{184,-30},{170,-30}},
                        color={0,127,255}));
    connect(sinZonFlo.ports[2], buiHea.ports_bHeaWat[2]) annotation (Line(
          points={{199.58,-29.2},{222,-29.2},{222,-30},{244,-30}},
                             color={0,127,255}));
    connect(sinZonFlo.weaBus, weaDat.weaBus) annotation (Line(
        points={{198.08,-11.8},{198,-11.8},{198,120},{230,120}},
        color={255,204,51},
        thickness=0.5));
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
              -46},{-58,-46},{-58,-32},{5.75,-32},{5.75,-35.0435}},
            color={0,127,255}));
      connect(buiHea.PPum, acLoad.Pow) annotation (Line(points={{63.875,-23.5652},
              {88,-23.5652},{88,-112}},
                                 color={0,0,127}));
      connect(buiHea.ports_bHeaWat[1], tan.fluPorVol[4]) annotation (Line(points={{62,
              -35.0435},{62,-68},{-160,-68},{-160,-12.16},{-128,-12.16}},
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
