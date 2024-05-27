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
    Buildings.Fluid.Actuators.Valves.ThreeWayLinear valLin(
      redeclare package Medium = Buildings.Media.Water,
      l={0.05,0.05},
      m_flow_nominal=2,
      use_inputFilter=false,
      dpValve_nominal=1,
      energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial)
      "Valve model, linear opening characteristics"
      annotation (Placement(transformation(extent={{-172,-150},{-152,-130}})));
    Buildings.Fluid.Actuators.Valves.ThreeWayLinear valLin1(
      redeclare package Medium = Buildings.Media.Water,
      l={0.05,0.05},
      m_flow_nominal=2,
      use_inputFilter=false,
      dpValve_nominal=1,
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
    Generators.Digital_Twins.NeoTower2_GC neoTower2_GC
      annotation (Placement(transformation(extent={{-524,-174},{-414,-98}})));
    Generators.Digital_Twins.WolfCGB14_GC wolfCGB14_GC
      annotation (Placement(transformation(extent={{-448,-2},{-354,52}})));
    Storage.StratifiedHeatStorage tan(
      redeclare package Medium = Buildings.Media.Water,
      m_flow_nominal=2,
      VTan(displayUnit="l") = 0.775,
      hTan=3,
      dIns=0.3,
      nSeg=10,
      T_start=333.15)
      annotation (Placement(transformation(extent={{-60,-56},{16,20}})));
    Fluid.Pumps.Test_Pump_controler test_Pump_controler
      annotation (Placement(transformation(extent={{-42,-264},{-22,-244}})));
    Modelica.Blocks.Logical.Or or1
      annotation (Placement(transformation(extent={{-622,-130},{-602,-110}})));
    Modelica.Blocks.Logical.Or or2
      annotation (Placement(transformation(extent={{-592,-16},{-572,4}})));
    Modelica.Blocks.Logical.LogicalDelay logicalDelay(delayTime=200)
      annotation (Placement(transformation(extent={{-668,-94},{-648,-74}})));
    Modelica.Blocks.Logical.LogicalDelay logicalDelay1(delayTime=200)
      annotation (Placement(transformation(extent={{-606,30},{-586,50}})));
    Modelica.Blocks.Logical.LessThreshold lessThreshold1(threshold=335)
      annotation (Placement(transformation(extent={{-650,24},{-630,44}})));
    Modelica.Blocks.Logical.LessThreshold lessThreshold4(threshold=335)
      annotation (Placement(transformation(extent={{-714,-94},{-694,-74}})));
    Modelica.Blocks.Logical.LogicalDelay logicalDelay2(delayTime=200)
      annotation (Placement(transformation(extent={{-662,-160},{-642,-140}})));
    Modelica.Blocks.Logical.LogicalDelay logicalDelay3(delayTime=200)
      annotation (Placement(transformation(extent={{-608,-54},{-588,-34}})));
    Modelica.Blocks.Logical.LessEqualThreshold lessEqualThreshold(threshold=343)
      annotation (Placement(transformation(extent={{-728,-156},{-708,-136}})));
    Modelica.Blocks.Logical.LessEqualThreshold lessEqualThreshold1(threshold=
          343)
      annotation (Placement(transformation(extent={{-666,-44},{-646,-24}})));
    Buildings.ThermalZones.Detailed.Validation.BaseClasses.SingleZoneFloor sinZonFlo(
      redeclare package Medium =
          Modelica.Media.Water.ConstantPropertyLiquidWater,
      use_windPressure=false,
      gai(K=0*[0.4; 0.4; 0.2]))
      "Single-zone floor model"
      annotation (Placement(transformation(extent={{200,-68},{224,-44}})));
    Modelica.Blocks.Math.Gain gaiHea(k=1E6) "Gain for heating"
      annotation (Placement(visible=true,transformation(origin={356,-2},
                                                                       extent={{-6,-6},{6,6}},rotation=0)));
    Buildings.Controls.Continuous.LimPID conHeaPID(
      Ti=300,
      k=0.1,
      reverseActing=true,
      strict=true) "Controller for heating"
      annotation (Placement(visible=true, transformation(origin={334,-2},extent={{-6,-6},{6,6}},rotation=0)));
    Modelica.Blocks.Sources.Constant TSetHea(k=273.15 + 20)
      "Set-point for heating"
      annotation (Placement(visible=true, transformation(origin={310,-2},extent={{-6,-6},{6,6}},rotation=0)));
    Modelica.Blocks.Sources.Constant TSetCoo(k=273.15 + 27)
      "Set-point for cooling"
      annotation (Placement(visible=true, transformation(origin={310,-28},
                                                                         extent={{-6,-6},{6,6}},rotation=0)));
    Modelica.Blocks.Math.Gain gaiCoo(k=-1E6) "Gain for cooling"
      annotation (Placement(visible=true,transformation(origin={356,-28},
                                                                       extent={{-6,-6},{6,6}},rotation=0)));
    Buildings.Controls.Continuous.LimPID conCooPID(
      Ti=300,
      k=0.1,
      reverseActing=false,
      strict=true) "Controller for cooling"
      annotation (Placement(visible=true, transformation(origin={334,-28},
                                                                         extent={{-6,-6},{6,6}},rotation=0)));
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
            -128}},                color={0,0,127}));
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
    connect(neoTower2_GC.weaBus, weaDat.weaBus) annotation (Line(
        points={{-455.938,-71.7455},{-455.938,120},{230,120}},
        color={255,204,51},
        thickness=0.5));
    connect(realExpression1.y, neoTower2_GC.CHPModulation) annotation (Line(
          points={{-537,-110},{-537,-114.236},{-505.094,-114.236}}, color={0,0,
            127}));
    connect(neoTower2_GC.port_b, valLin.port_3) annotation (Line(points={{-436,
            -122.873},{-192,-122.873},{-192,-150},{-162,-150}}, color={0,127,
            255}));
    connect(neoTower2_GC.port_a, valLin1.port_3) annotation (Line(points={{
            -434.625,-144.291},{-332,-144.291},{-332,-40},{-234,-40},{-234,-26}},
          color={0,127,255}));
    connect(neoTower2_GC.term_p, gri.terminal) annotation (Line(points={{
            -456.625,-158.109},{-456.625,-410},{82,-410}}, color={0,120,120}));
    connect(realExpression.y, wolfCGB14_GC.ControlIn) annotation (Line(points={{-511,16},
            {-460,16},{-460,38.2545},{-440.579,38.2545}},           color={0,0,
            127}));
    connect(wolfCGB14_GC.weaBus, weaDat.weaBus) annotation (Line(
        points={{-410.4,47.5818},{-410.4,120},{230,120}},
        color={255,204,51},
        thickness=0.5));
    connect(wolfCGB14_GC.port_b, valLin.port_1) annotation (Line(points={{
            -369.337,38.7455},{-369.337,60},{-304,60},{-304,-120},{-180,-120},{
            -180,-140},{-172,-140}}, color={0,127,255}));
    connect(wolfCGB14_GC.port_a, valLin1.port_1) annotation (Line(points={{
            -362.905,18.6182},{-256,18.6182},{-256,-16},{-244,-16}}, color={0,
            127,255}));
    connect(wolfCGB14_GC.term_p, gri.terminal) annotation (Line(points={{
            -398.526,-9.36364},{-398.526,-410},{82,-410}}, color={0,120,120}));
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
    connect(valLin1.port_2, tan.fluPorVol[2]) annotation (Line(points={{-224,
            -16},{-132.5,-16},{-132.5,-23.32},{-41,-23.32}}, color={0,127,255}));
    connect(or1.y, neoTower2_GC.CHPOn) annotation (Line(points={{-601,-120},{
            -572,-120},{-572,-72},{-508,-72},{-508,-76},{-505.094,-76},{
            -505.094,-91.4364}}, color={255,0,255}));
    connect(or2.y, wolfCGB14_GC.CBOn1) annotation (Line(points={{-571,-6},{-571,
            -8},{-540,-8},{-540,36},{-464,36},{-464,25.9818},{-443.053,25.9818}},
          color={255,0,255}));
    connect(logicalDelay1.y1, or2.u1) annotation (Line(points={{-585,46},{-576,
            46},{-576,12},{-608,12},{-608,-6},{-594,-6}}, color={255,0,255}));
    connect(logicalDelay.y2, or1.u1) annotation (Line(points={{-647,-90},{-647,
            -92},{-636,-92},{-636,-120},{-624,-120}},              color={255,0,
            255}));
    connect(lessThreshold1.y, logicalDelay1.u) annotation (Line(points={{-629,
            34},{-622,34},{-622,40},{-608,40}}, color={255,0,255}));
    connect(lessThreshold1.u, tan.TempTop) annotation (Line(points={{-652,34},{
            -668,34},{-668,88},{28,88},{28,1},{8.4,1}}, color={0,0,127}));
    connect(lessThreshold4.y, logicalDelay.u) annotation (Line(points={{-693,
            -84},{-670,-84}},                       color={255,0,255}));
    connect(lessThreshold4.u, tan.TempTop) annotation (Line(points={{-716,-84},
            {-724,-84},{-724,32},{-668,32},{-668,88},{28,88},{28,1},{8.4,1}},
          color={0,0,127}));
    connect(logicalDelay2.y1, or1.u2) annotation (Line(points={{-641,-144},{
            -636,-144},{-636,-128},{-624,-128}}, color={255,0,255}));
    connect(logicalDelay3.y1, or2.u2) annotation (Line(points={{-587,-38},{-587,
            -28},{-594,-28},{-594,-14}}, color={255,0,255}));
    connect(buiHea.ports_aHeaWat[1], tan.fluPorVol1[2]) annotation (Line(points={{174,
            -59.4667},{144,-59.4667},{144,-12},{-8,-12},{-8,-22},{-8.32,-22},{
            -8.32,-22.56}},
          color={0,127,255}));
    connect(valLin2.port_2, tan.fluPorVol1[8]) annotation (Line(points={{110,
            -124},{38,-124},{38,-26},{-8,-26},{-8,-14},{-8.32,-14},{-8.32,
            -13.44}},                                    color={0,127,255}));
    connect(valLin.port_2, tan.fluPorVol[10]) annotation (Line(points={{-152,
            -140},{-144,-140},{-144,-11.16},{-41,-11.16}}, color={0,127,255}));
    connect(lessEqualThreshold.y, logicalDelay2.u) annotation (Line(points={{-707,
            -146},{-707,-150},{-664,-150}},                              color=
            {255,0,255}));
    connect(lessEqualThreshold1.y, logicalDelay3.u) annotation (Line(points={{
            -645,-34},{-645,-36},{-620,-36},{-620,-44},{-610,-44}}, color={255,
            0,255}));
    connect(tan.Tempbot, lessEqualThreshold.u) annotation (Line(points={{8.4,
            -38.52},{28,-38.52},{28,-68},{-444,-68},{-444,-80},{-488,-80},{-488,
            -76},{-568,-76},{-568,-172},{-740,-172},{-740,-146},{-730,-146}},
          color={0,0,127}));
    connect(tan.Tempbot, lessEqualThreshold1.u) annotation (Line(points={{8.4,
            -38.52},{28,-38.52},{28,-68},{-444,-68},{-444,-80},{-488,-80},{-488,
            -76},{-580,-76},{-580,-64},{-676,-64},{-676,-34},{-668,-34}}, color
          ={0,0,127}));
    connect(conHeaPID.y,gaiHea. u)
      annotation (Line(points={{340.6,-2},{348.8,-2}},
                                                    color={0,0,127}));
    connect(TSetHea.y,conHeaPID. u_s)
      annotation (Line(points={{316.6,-2},{326.8,-2}}, color={0,0,127}));
    connect(conCooPID.u_s,TSetCoo. y)
      annotation (Line(points={{326.8,-28},{316.6,-28}},
                                                       color={0,0,127}));
    connect(conCooPID.y,gaiCoo. u)
      annotation (Line(points={{340.6,-28},{348.8,-28}},
                                                    color={0,0,127}));
    connect(conHeaPID.y,gaiHea. u)
      annotation (Line(points={{340.6,-2},{348.8,-2}},
                                                    color={0,0,127}));
    connect(gaiCoo.u,conCooPID. y)
      annotation (Line(points={{348.8,-28},{340.6,-28}},
                                                    color={0,0,127}));
    connect(sinZonFlo.TRooAir,conCooPID. u_m) annotation (Line(points={{222.2,
            -49.88},{292,-49.88},{292,-35.2},{334,-35.2}},        color={0,0,
            127}));
    connect(sinZonFlo.TRooAir,conHeaPID. u_m) annotation (Line(points={{222.2,
            -49.88},{292,-49.88},{292,-9.2},{334,-9.2}},
          color={0,0,127}));
    connect(sinZonFlo.ports[1], buiHea.ports_aHeaWat[2]) annotation (Line(
          points={{204.98,-63.2},{204.98,-57.3333},{174,-57.3333}}, color={0,
            127,255}));
    connect(sinZonFlo.ports[2], buiHea.ports_bHeaWat[2]) annotation (Line(
          points={{205.58,-63.2},{226.64,-63.2},{226.64,-57.3333},{248,-57.3333}},
          color={0,127,255}));
    connect(weaDat.weaBus, sinZonFlo.weaBus) annotation (Line(
        points={{230,120},{204.08,120},{204.08,-45.8}},
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
        StopTime=10000,
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

  model SF1_exp "Example model of a building with loads provided as time series and
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
      annotation (Placement(transformation(extent={{-490,48},{-470,68}})));
    Modelica.Blocks.Sources.RealExpression realExpression1(y=1)
      annotation (Placement(transformation(extent={{-558,-120},{-538,-100}})));
    Buildings.Fluid.Actuators.Valves.ThreeWayLinear valLin(
      redeclare package Medium = Buildings.Media.Water,
      l={0.05,0.05},
      m_flow_nominal=2,
      use_inputFilter=false,
      dpValve_nominal=1,
      energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial)
      "Valve model, linear opening characteristics"
      annotation (Placement(transformation(extent={{-172,-150},{-152,-130}})));
    Buildings.Fluid.Actuators.Valves.ThreeWayLinear valLin1(
      redeclare package Medium = Buildings.Media.Water,
      l={0.05,0.05},
      m_flow_nominal=2,
      use_inputFilter=false,
      dpValve_nominal=1,
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
    Buildings.Controls.OBC.CDL.Conversions.BooleanToReal booToRea(realTrue=
          0.5)
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
      annotation (Placement(transformation(extent={{104,10},{124,30}})));
    Generators.Digital_Twins.NeoTower2_GC neoTower2_GC
      annotation (Placement(transformation(extent={{-524,-174},{-414,-98}})));
    Generators.Digital_Twins.WolfCGB14_GC wolfCGB14_GC
      annotation (Placement(transformation(extent={{-448,-2},{-354,52}})));
    Storage.StratifiedHeatStorage tan(
      redeclare package Medium = Buildings.Media.Water,
      m_flow_nominal=2,
      VTan(displayUnit="l") = 0.775,
      hTan=3,
      dIns=0.3,
      nSeg=10,
      T_start=333.15)
      annotation (Placement(transformation(extent={{-60,-56},{16,20}})));
    Fluid.Pumps.Test_Pump_controler test_Pump_controler
      annotation (Placement(transformation(extent={{-42,-264},{-22,-244}})));
    Buildings.DHC.Loads.HotWater.BaseClasses.TankChargingController tanCha
      annotation (Placement(transformation(extent={{-688,-86},{-618,-8}})));
    Modelica.Blocks.Sources.RealExpression realExpression2(y=343)
      annotation (Placement(transformation(extent={{-750,-26},{-730,-6}})));
    Buildings.ThermalZones.Detailed.Validation.BaseClasses.SingleZoneFloor sinZonFlo(
      redeclare package Medium =
          Modelica.Media.Water.ConstantPropertyLiquidWater,
      use_windPressure=false,
      gai(K=0*[0.4; 0.4; 0.2]))
      "Single-zone floor model"
      annotation (Placement(transformation(extent={{198,-68},{222,-44}})));
    Modelica.Blocks.Math.Gain gaiHea(k=1E6) "Gain for heating"
      annotation (Placement(visible=true,transformation(origin={364,-4},
                                                                       extent={{-6,-6},{6,6}},rotation=0)));
    Buildings.Controls.Continuous.LimPID conHeaPID(
      Ti=300,
      k=0.1,
      reverseActing=true,
      strict=true) "Controller for heating"
      annotation (Placement(visible=true, transformation(origin={342,-4},extent={{-6,-6},{6,6}},rotation=0)));
    Modelica.Blocks.Sources.Constant TSetHea(k=273.15 + 20)
      "Set-point for heating"
      annotation (Placement(visible=true, transformation(origin={318,-4},extent={{-6,-6},{6,6}},rotation=0)));
    Modelica.Blocks.Sources.Constant TSetCoo(k=273.15 + 27)
      "Set-point for cooling"
      annotation (Placement(visible=true, transformation(origin={318,-30},
                                                                         extent={{-6,-6},{6,6}},rotation=0)));
    Modelica.Blocks.Math.Gain gaiCoo(k=-1E6) "Gain for cooling"
      annotation (Placement(visible=true,transformation(origin={364,-30},
                                                                       extent={{-6,-6},{6,6}},rotation=0)));
    Buildings.Controls.Continuous.LimPID conCooPID(
      Ti=300,
      k=0.1,
      reverseActing=false,
      strict=true) "Controller for cooling"
      annotation (Placement(visible=true, transformation(origin={342,-30},
                                                                         extent={{-6,-6},{6,6}},rotation=0)));
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
            -128}},                color={0,0,127}));
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
    connect(y1.y, valLin2.y) annotation (Line(points={{125,20},{134,20},{134,
            -136},{120,-136}},            color={0,0,127}));
    connect(neoTower2_GC.weaBus, weaDat.weaBus) annotation (Line(
        points={{-455.938,-71.7455},{-455.938,-72},{-460,-72},{-460,120},{230,
            120}},
        color={255,204,51},
        thickness=0.5));
    connect(realExpression1.y, neoTower2_GC.CHPModulation) annotation (Line(
          points={{-537,-110},{-537,-114.236},{-505.094,-114.236}}, color={0,0,
            127}));
    connect(neoTower2_GC.port_b, valLin.port_3) annotation (Line(points={{-436,
            -122.873},{-436,-124},{-184,-124},{-184,-150},{-162,-150}},
                                                                color={0,127,
            255}));
    connect(neoTower2_GC.port_a, valLin1.port_3) annotation (Line(points={{
            -434.625,-144.291},{-334,-144.291},{-334,-144},{-234,-144},{-234,
            -26}},
          color={0,127,255}));
    connect(neoTower2_GC.term_p, gri.terminal) annotation (Line(points={{
            -456.625,-158.109},{-456.625,-410},{82,-410}}, color={0,120,120}));
    connect(realExpression.y, wolfCGB14_GC.ControlIn) annotation (Line(points={{-469,58},
            {-460,58},{-460,38.2545},{-440.579,38.2545}},           color={0,0,
            127}));
    connect(wolfCGB14_GC.weaBus, weaDat.weaBus) annotation (Line(
        points={{-410.4,47.5818},{-410.4,120},{230,120}},
        color={255,204,51},
        thickness=0.5));
    connect(wolfCGB14_GC.port_b, valLin.port_1) annotation (Line(points={{
            -369.337,38.7455},{-369.337,60},{-304,60},{-304,-120},{-180,-120},{
            -180,-140},{-172,-140}}, color={0,127,255}));
    connect(wolfCGB14_GC.port_a, valLin1.port_1) annotation (Line(points={{
            -362.905,18.6182},{-362,18.6182},{-362,-16},{-244,-16}}, color={0,
            127,255}));
    connect(wolfCGB14_GC.term_p, gri.terminal) annotation (Line(points={{
            -398.526,-9.36364},{-398.526,-410},{82,-410}}, color={0,120,120}));
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
    connect(valLin1.port_2, tan.fluPorVol[2]) annotation (Line(points={{-224,
            -16},{-42,-16},{-42,-23.32},{-41,-23.32}},       color={0,127,255}));
    connect(buiHea.ports_aHeaWat[1], tan.fluPorVol1[2]) annotation (Line(points={{174,
            -59.4667},{144,-59.4667},{144,-10},{-8,-10},{-8,-22},{-8.32,-22},{
            -8.32,-22.56}},
          color={0,127,255}));
    connect(valLin2.port_2, tan.fluPorVol1[8]) annotation (Line(points={{110,
            -124},{36,-124},{36,-24},{-4,-24},{-4,-13.44},{-8.32,-13.44}},
                                                         color={0,127,255}));
    connect(valLin.port_2, tan.fluPorVol[10]) annotation (Line(points={{-152,
            -140},{-148,-140},{-148,-28},{-42,-28},{-42,-11.16},{-41,-11.16}},
                                                           color={0,127,255}));
    connect(tanCha.charge, neoTower2_GC.CHPOn) annotation (Line(points={{-611,
            -47},{-512,-47},{-512,-90},{-505.094,-90},{-505.094,-91.4364}},
          color={255,0,255}));
    connect(tanCha.TTanTop, tan.TempTop) annotation (Line(points={{-695,-47},
            {-712,-47},{-712,84},{28,84},{28,1},{8.4,1}}, color={0,0,127}));
    connect(tanCha.TTanBot, tan.Tempbot) annotation (Line(points={{-695,-78.2},
            {-712,-78.2},{-712,-78},{-716,-78},{-716,-170},{26,-170},{26,
            -38.52},{8.4,-38.52}}, color={0,0,127}));
    connect(realExpression2.y, tanCha.TTanTopSet) annotation (Line(points={{
            -729,-16},{-708.25,-16},{-708.25,-15.8},{-691.5,-15.8}}, color={0,
            0,127}));
    connect(tanCha.charge, wolfCGB14_GC.CBOn1) annotation (Line(points={{-611,
            -47},{-512,-47},{-512,26},{-464,26},{-464,25.9818},{-443.053,
            25.9818}}, color={255,0,255}));
    connect(sinZonFlo.ports[1], buiHea.ports_aHeaWat[2]) annotation (Line(
          points={{202.98,-63.2},{202.98,-57.3333},{174,-57.3333}}, color={0,
            127,255}));
    connect(sinZonFlo.ports[2], buiHea.ports_bHeaWat[2]) annotation (Line(
          points={{203.58,-63.2},{224.64,-63.2},{224.64,-57.3333},{248,-57.3333}},
          color={0,127,255}));
    connect(sinZonFlo.weaBus, weaDat.weaBus) annotation (Line(
        points={{202.08,-45.8},{202.08,120},{230,120}},
        color={255,204,51},
        thickness=0.5));
    connect(conHeaPID.y,gaiHea. u)
      annotation (Line(points={{348.6,-4},{356.8,-4}},
                                                    color={0,0,127}));
    connect(TSetHea.y,conHeaPID. u_s)
      annotation (Line(points={{324.6,-4},{334.8,-4}}, color={0,0,127}));
    connect(conCooPID.u_s,TSetCoo. y)
      annotation (Line(points={{334.8,-30},{324.6,-30}},
                                                       color={0,0,127}));
    connect(conCooPID.y,gaiCoo. u)
      annotation (Line(points={{348.6,-30},{356.8,-30}},
                                                    color={0,0,127}));
    connect(conHeaPID.y,gaiHea. u)
      annotation (Line(points={{348.6,-4},{356.8,-4}},
                                                    color={0,0,127}));
    connect(gaiCoo.u,conCooPID. y)
      annotation (Line(points={{356.8,-30},{348.6,-30}},
                                                    color={0,0,127}));
    connect(sinZonFlo.TRooAir,conHeaPID. u_m) annotation (Line(points={{220.2,
            -49.88},{220.2,16},{342,16},{342,-11.2}},
          color={0,0,127}));
    connect(sinZonFlo.TRooAir, conCooPID.u_m) annotation (Line(points={{220.2,
            -49.88},{220.2,-8},{300,-8},{300,-37.2},{342,-37.2}}, color={0,0,
            127}));
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
        StopTime=10000,
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
  end SF1_exp;

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
      annotation (Placement(transformation(extent={{214,-168},{234,-148}})));
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
          origin={226,-114})));

    Buildings.DHC.Loads.HotWater.ThermostaticMixingValve theMixVal(redeclare
        package Medium = Medium, mMix_flow_nominal=1.2*datWatHea.mDom_flow_nominal)
      annotation (Placement(transformation(extent={{248,-216},{292,-178}})));
    Modelica.Blocks.Sources.Constant conTSetMix(k(
        final unit="K",
        displayUnit="degC") = 308.15)
      "Temperature setpoint for mixed water supply to fixture"
      annotation (Placement(transformation(extent={{-11,-11},{11,11}},
          rotation=180,
          origin={227,-79})));
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
    Modelica.Blocks.Sources.Ramp y2(
      height=1,
      duration=1,
      offset=0) "Control signal"
      annotation (Placement(transformation(extent={{-260,-258},{-240,-238}})));

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
    Consumers.DHW domHotWatTan(
      redeclare package MediumDom = Buildings.Media.Water,
      redeclare package MediumHea = Buildings.Media.Water,
      dat=datWatHea)
      annotation (Placement(transformation(extent={{132,-302},{202,-236}})));
    Buildings.DHC.Loads.BaseClasses.BuildingTimeSeries buiHea(
      have_chiWat=false,
      filNam=
          "modelica://Buildings/Resources/Data/DHC/Loads/Examples/SwissResidential_20190916.mos",
      nPorts_aChiWat=1,
      nPorts_bChiWat=1,
      nPorts_aHeaWat=2,
      nPorts_bHeaWat=2)
      "Building with heating only"
      annotation (Placement(transformation(extent={{212,-26},{286,38}})));

    Buildings.ThermalZones.Detailed.Validation.BaseClasses.SingleZoneFloor sinZonFlo(
      redeclare package Medium =
          Modelica.Media.Water.ConstantPropertyLiquidWater,
      use_windPressure=false,
      gai(K=0*[0.4; 0.4; 0.2]))
      "Single-zone floor model"
      annotation (Placement(transformation(extent={{238,-10},{262,14}})));
    Modelica.Blocks.Math.Gain gaiHea(k=1E6) "Gain for heating"
      annotation (Placement(visible=true,transformation(origin={448,38},
                                                                       extent={{-6,-6},{6,6}},rotation=0)));
    Buildings.Controls.Continuous.LimPID conHeaPID(
      Ti=300,
      k=0.1,
      reverseActing=true,
      strict=true) "Controller for heating"
      annotation (Placement(visible=true, transformation(origin={426,38},extent={{-6,-6},{6,6}},rotation=0)));
    Modelica.Blocks.Sources.Constant TSetHea(k=273.15 + 20)
      "Set-point for heating"
      annotation (Placement(visible=true, transformation(origin={402,38},extent={{-6,-6},{6,6}},rotation=0)));
    Modelica.Blocks.Sources.Constant TSetCoo(k=273.15 + 27)
      "Set-point for cooling"
      annotation (Placement(visible=true, transformation(origin={402,12},extent={{-6,-6},{6,6}},rotation=0)));
    Modelica.Blocks.Math.Gain gaiCoo(k=-1E6) "Gain for cooling"
      annotation (Placement(visible=true,transformation(origin={448,12},
                                                                       extent={{-6,-6},{6,6}},rotation=0)));
    Buildings.Controls.Continuous.LimPID conCooPID(
      Ti=300,
      k=0.1,
      reverseActing=false,
      strict=true) "Controller for cooling"
      annotation (Placement(visible=true, transformation(origin={426,12},extent={{-6,-6},{6,6}},rotation=0)));
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
            {200,-181.8},{200,-114},{212.8,-114}},
                                color={0,0,127}));
    connect(theMixVal.port_col, souCol.ports[1]) annotation (Line(points={{248,
            -212.2},{126,-212.2},{126,-184.7}},                   color={0,127,
            255}));
    connect(conTSetMix.y, theMixVal.TMixSet) annotation (Line(points={{214.9,
            -79},{192,-79},{192,-193.2},{245.8,-193.2}},   color={0,0,127}));
    connect(acLoad1.terminal, gri.terminal) annotation (Line(points={{250,-372},
            {250,-410},{82,-410}}, color={0,120,120}));
    connect(lin.terminal_p, pv.terminal) annotation (Line(points={{104,-489},{
            428,-489},{428,-397},{414,-397}}, color={0,0,255}));
    connect(lin.terminal_n, conv.terminal_p) annotation (Line(points={{54,-489},
            {-44,-489},{-44,-451},{-30,-451}},                       color={0,0,
            255}));
    connect(y.y, valLin2.y) annotation (Line(points={{-271,66},{-176,66},{-176,
            12}}, color={0,0,127}));
    connect(solCol.weaBus, weaDat.weaBus) annotation (Line(
        points={{-338,-107.08},{-360,-107.08},{-360,120},{230,120}},
        color={255,204,51},
        thickness=0.5));
    connect(y.y, valLin1.y) annotation (Line(points={{-271,66},{-176,66},{-176,
            64},{-120,64},{-120,-84}}, color={0,0,127}));
    connect(y2.y, valLin3.y) annotation (Line(points={{-239,-248},{-200,-248},{
            -200,-262}}, color={0,0,127}));
    connect(valLin4.y, y2.y) annotation (Line(points={{-126,-298},{-128,-298},{
            -128,-248},{-239,-248}},             color={0,0,127}));
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

    connect(realExpression.y, wolfCGB20_GC.ControlIn) annotation (Line(points={{-493,
            -14},{-493,-16.19},{-461.007,-16.19}},       color={0,0,127}));
    connect(wolfCGB20_GC.term_p, gri.terminal) annotation (Line(points={{
            -414.267,-46.7},{-414.267,-64},{-414,-64},{-414,-406},{-366,-406},{
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
    connect(realExpression1.y, test_Pump_controler.SetInSignal) annotation (
        Line(points={{-189,-118},{-196,-118},{-196,-131},{-198.2,-131}}, color=
            {0,0,127}));
    connect(test_Pump_controler.term_p, gri.terminal) annotation (Line(points={
            {-184.4,-155.4},{-184.4,-154},{-164,-154},{-164,-410},{82,-410}},
          color={0,120,120}));
    connect(souCol.ports[2], domHotWatTan.port_aDom) annotation (Line(points={{
            126,-181.3},{126,-228},{132,-228},{132,-249.2}}, color={0,127,255}));
    connect(domHotWatTan.port_bDom, theMixVal.port_hot) annotation (Line(points
          ={{202,-249.2},{228,-249.2},{228,-204.6},{248,-204.6}}, color={0,127,
            255}));
    connect(conTSetHot.y, domHotWatTan.TDomSet) annotation (Line(points={{114.7,
            -259},{114.7,-269},{128.5,-269}}, color={0,0,127}));
    connect(domHotWatTan.PEle, acLoad1.Pow) annotation (Line(points={{205.5,
            -269},{250,-269},{250,-326}}, color={0,0,127}));
    connect(solCol.port_a, tan.port_b) annotation (Line(points={{-338,-133},{
            -344,-133},{-344,-190},{8,-190},{8,-42}}, color={0,127,255}));
    connect(valLin3.port_2, valLin1.port_1) annotation (Line(points={{-190,-274},
            {-184,-274},{-184,-188},{-156,-188},{-156,-76},{-130,-76},{-130,-96}},
          color={0,127,255}));
    connect(valLin4.port_3, valLin2.port_3) annotation (Line(points={{-126,-320},
            {-128,-320},{-128,-328},{-152,-328},{-152,-24},{-176,-24},{-176,-10}},
          color={0,127,255}));
    connect(buiHea.PPum, acLoad.Pow) annotation (Line(points={{288.467,12.4},{
            316,12.4},{316,-328}}, color={0,0,127}));
    connect(sinZonFlo.weaBus, weaDat.weaBus) annotation (Line(
        points={{242.08,12.2},{242.08,120},{230,120}},
        color={255,204,51},
        thickness=0.5));
    connect(conHeaPID.y,gaiHea. u)
      annotation (Line(points={{432.6,38},{440.8,38}},
                                                    color={0,0,127}));
    connect(TSetHea.y,conHeaPID. u_s)
      annotation (Line(points={{408.6,38},{418.8,38}}, color={0,0,127}));
    connect(conCooPID.u_s,TSetCoo. y)
      annotation (Line(points={{418.8,12},{408.6,12}}, color={0,0,127}));
    connect(conCooPID.y,gaiCoo. u)
      annotation (Line(points={{432.6,12},{440.8,12}},
                                                    color={0,0,127}));
    connect(conHeaPID.y,gaiHea. u)
      annotation (Line(points={{432.6,38},{440.8,38}},
                                                    color={0,0,127}));
    connect(gaiCoo.u,conCooPID. y)
      annotation (Line(points={{440.8,12},{432.6,12}},
                                                    color={0,0,127}));
    connect(sinZonFlo.TRooAir,conCooPID. u_m) annotation (Line(points={{260.2,
            8.12},{388,8.12},{388,4.8},{426,4.8}},                color={0,0,
            127}));
    connect(sinZonFlo.TRooAir,conHeaPID. u_m) annotation (Line(points={{260.2,
            8.12},{384,8.12},{384,52},{426,52},{426,30.8}},
          color={0,0,127}));
    connect(test_Pump_controler.port_b, tan.port_a) annotation (Line(points={{
            -179.6,-141.8},{-106,-141.8},{-106,-142},{-34,-142},{-34,48},{8,48},
            {8,34}}, color={0,127,255}));
    connect(domHotWatTan.port_aHea, tan.fluPorVol1[8]) annotation (Line(points=
            {{202,-288.8},{216,-288.8},{216,-316},{48,-316},{48,-14},{22,-14},{
            22,0},{21.68,0},{21.68,0.56}}, color={0,127,255}));
    connect(domHotWatTan.port_bHea, tan.fluPorVol1[2]) annotation (Line(points=
            {{132,-288.8},{54,-288.8},{54,4},{22,4},{22,-8},{21.68,-8},{21.68,
            -8.56}}, color={0,127,255}));
    connect(sinZonFlo.ports[1], buiHea.ports_aHeaWat[1]) annotation (Line(
          points={{242.98,-5.2},{242.98,-1.46667},{212,-1.46667}}, color={0,127,
            255}));
    connect(sinZonFlo.ports[2], buiHea.ports_bHeaWat[1]) annotation (Line(
          points={{243.58,-5.2},{265.64,-5.2},{265.64,-1.46667},{286,-1.46667}},
          color={0,127,255}));
    connect(buiHea.ports_bHeaWat[2], tan.fluPorVol1[7]) annotation (Line(points
          ={{286,0.666667},{302,0.666667},{302,-46},{70,-46},{70,-10},{24,-10},
            {24,0},{21.68,0},{21.68,-0.96}}, color={0,127,255}));
    connect(buiHea.ports_aHeaWat[2], tan.fluPorVol1[4]) annotation (Line(points
          ={{212,0.666667},{22,0.666667},{22,-2},{21.68,-2},{21.68,-5.52}},
          color={0,127,255}));
    connect(valLin2.port_2, tan.fluPorVol[2]) annotation (Line(points={{-166,0},
            {-14,0},{-14,2},{-12,2},{-12,-9.32},{-11,-9.32}}, color={0,127,255}));
    connect(valLin3.port_3, tan.fluPorVol[4]) annotation (Line(points={{-200,
            -284},{-200,-288},{-106,-288},{-106,-8},{-12,-8},{-12,-6.28},{-11,
            -6.28}}, color={0,127,255}));
    connect(valLin4.port_2, tan.fluPorVol[6]) annotation (Line(points={{-116,
            -310},{-116,-312},{-96,-312},{-96,-14},{-10,-14},{-10,-12},{-11,-12},
            {-11,-3.24}}, color={0,127,255}));
    connect(valLin1.port_2, tan.fluPorVol[8]) annotation (Line(points={{-110,
            -96},{-82,-96},{-82,-18},{-11,-18},{-11,-0.2}}, color={0,127,255}));
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
        StopTime=9000,
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

  model SF3 "Example model of a building with loads provided as time series and
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
      annotation (Placement(transformation(extent={{-532,-50},{-512,-30}})));
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

    Generators.Digital_Twins.WolfBWS1_10_GC wolfBWS1_10_GC
      annotation (Placement(transformation(extent={{-450,-54},{-374,22}})));
    Storage.StratifiedHeatStorage tan1(
      redeclare package Medium = Buildings.Media.Water,
      m_flow_nominal=0.1,
      VTan(displayUnit="l") = 0.775,
      hTan=3,
      dIns=0.3,
      nSeg=10)
      annotation (Placement(transformation(extent={{-70,-64},{6,12}})));
    Fluid.Valves.TwoWayEqualPercentage val(
      redeclare package Medium = ProsNet.Media.Water,
      m_flow_nominal=10,
      dpValve_nominal=6000)
      annotation (Placement(transformation(extent={{-258,-76},{-238,-56}})));
    Consumers.DHW domHotWatTan(
      redeclare package MediumDom = Buildings.Media.Water,
      redeclare package MediumHea = Buildings.Media.Water,
      dat=datWatHea)
      annotation (Placement(transformation(extent={{146,-300},{216,-234}})));
    Fluid.Pumps.Test_Pump_controler test_Pump_controler
      annotation (Placement(transformation(extent={{-168,-306},{-148,-286}})));
    Buildings.ThermalZones.Detailed.Validation.BaseClasses.SingleZoneFloor sinZonFlo(
      redeclare package Medium =
          Modelica.Media.Water.ConstantPropertyLiquidWater,
      use_windPressure=false,
      gai(K=0*[0.4; 0.4; 0.2]))
      "Single-zone floor model"
      annotation (Placement(transformation(extent={{208,-18},{232,6}})));
    Modelica.Blocks.Math.Gain gaiHea(k=1E6) "Gain for heating"
      annotation (Placement(visible=true,transformation(origin={462,26},
                                                                       extent={{-6,-6},{6,6}},rotation=0)));
    Buildings.Controls.Continuous.LimPID conHeaPID(
      Ti=300,
      k=0.1,
      reverseActing=true,
      strict=true) "Controller for heating"
      annotation (Placement(visible=true, transformation(origin={440,26},extent={{-6,-6},{6,6}},rotation=0)));
    Modelica.Blocks.Sources.Constant TSetHea(k=273.15 + 20)
      "Set-point for heating"
      annotation (Placement(visible=true, transformation(origin={416,26},extent={{-6,-6},{6,6}},rotation=0)));
    Modelica.Blocks.Sources.Constant TSetCoo(k=273.15 + 27)
      "Set-point for cooling"
      annotation (Placement(visible=true, transformation(origin={416,0}, extent={{-6,-6},{6,6}},rotation=0)));
    Modelica.Blocks.Math.Gain gaiCoo(k=-1E6) "Gain for cooling"
      annotation (Placement(visible=true,transformation(origin={462,0},extent={{-6,-6},{6,6}},rotation=0)));
    Buildings.Controls.Continuous.LimPID conCooPID(
      Ti=300,
      k=0.1,
      reverseActing=false,
      strict=true) "Controller for cooling"
      annotation (Placement(visible=true, transformation(origin={440,0}, extent={{-6,-6},{6,6}},rotation=0)));
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
    connect(solCol.weaBus, weaDat.weaBus) annotation (Line(
        points={{-322,-271.08},{-322,120},{230,120}},
        color={255,204,51},
        thickness=0.5));
    connect(acLoad.Pow, buiHea.PPum) annotation (Line(points={{300,-328},{300,
            -228},{304,-228},{304,4},{260.467,4}}, color={0,0,127}));
    connect(wolfBWS1_10_GC.HPAuxModulation, realExpression.y) annotation (Line(
          points={{-448.86,-37.66},{-452,-37.66},{-452,-40},{-511,-40}}, color=
            {0,0,127}));
    connect(wolfBWS1_10_GC.HPOn, uHea.y) annotation (Line(points={{-448.86,0.34},
            {-448.86,2},{-494.4,2}}, color={255,0,255}));
    connect(wolfBWS1_10_GC.weaBus, weaDat.weaBus) annotation (Line(
        points={{-407.44,30.36},{-407.44,120},{230,120}},
        color={255,204,51},
        thickness=0.5));
    connect(wolfBWS1_10_GC.term_p, gri.terminal) annotation (Line(points={{
            -390.72,-66.92},{-390.72,-410},{82,-410}}, color={0,120,120}));
    connect(solCol.port_a, tan1.port_b) annotation (Line(points={{-322,-297},{
            -332,-297},{-332,-364},{-32,-364},{-32,-64}}, color={0,127,255}));
    connect(val.port_a, wolfBWS1_10_GC.port_b) annotation (Line(points={{-258,
            -66},{-344,-66},{-344,8.32},{-336,8.32}}, color={0,127,255}));
    connect(val.y, y.y) annotation (Line(points={{-248,-54},{-248,66},{-271,66}},
          color={0,0,127}));
    connect(domHotWatTan.port_aDom, souCol.ports[2]) annotation (Line(points={{
            146,-247.2},{144,-247.2},{144,-205.3},{124,-205.3}}, color={0,127,
            255}));
    connect(domHotWatTan.port_bDom, theMixVal.port_hot) annotation (Line(points
          ={{216,-247.2},{228,-247.2},{228,-204.6},{248,-204.6}}, color={0,127,
            255}));
    connect(domHotWatTan.PEle, acLoad1.Pow) annotation (Line(points={{219.5,
            -267},{250,-267},{250,-326}}, color={0,0,127}));
    connect(conTSetHot.y, domHotWatTan.TDomSet) annotation (Line(points={{114.7,
            -259},{114.7,-267},{142.5,-267}}, color={0,0,127}));
    connect(solCol.port_b, test_Pump_controler.port_a) annotation (Line(points=
            {{-262,-297},{-214.8,-297},{-214.8,-295.8},{-167.6,-295.8}}, color=
            {0,127,255}));
    connect(test_Pump_controler.port_b, tan1.port_a) annotation (Line(points={{
            -147.6,-295.8},{-102,-295.8},{-102,20},{-32,20},{-32,12}}, color={0,
            127,255}));
    connect(test_Pump_controler.term_p, gri.terminal) annotation (Line(points={
            {-152.4,-309.4},{-152.4,-410},{82,-410}}, color={0,120,120}));
    connect(realExpression1.y, test_Pump_controler.SetInSignal) annotation (
        Line(points={{-195,-282},{-195,-285},{-166.2,-285}}, color={0,0,127}));
    connect(sinZonFlo.weaBus, weaDat.weaBus) annotation (Line(
        points={{212.08,4.2},{212.08,104},{240,104},{240,120},{230,120}},
        color={255,204,51},
        thickness=0.5));
    connect(conHeaPID.y,gaiHea. u)
      annotation (Line(points={{446.6,26},{454.8,26}},
                                                    color={0,0,127}));
    connect(TSetHea.y,conHeaPID. u_s)
      annotation (Line(points={{422.6,26},{432.8,26}}, color={0,0,127}));
    connect(conCooPID.u_s,TSetCoo. y)
      annotation (Line(points={{432.8,0},{422.6,0}},   color={0,0,127}));
    connect(conCooPID.y,gaiCoo. u)
      annotation (Line(points={{446.6,0},{454.8,0}},color={0,0,127}));
    connect(conHeaPID.y,gaiHea. u)
      annotation (Line(points={{446.6,26},{454.8,26}},
                                                    color={0,0,127}));
    connect(gaiCoo.u,conCooPID. y)
      annotation (Line(points={{454.8,0},{446.6,0}},color={0,0,127}));
    connect(sinZonFlo.TRooAir,conCooPID. u_m) annotation (Line(points={{230.2,
            0.12},{400,0.12},{400,-7.2},{440,-7.2}},              color={0,0,
            127}));
    connect(sinZonFlo.TRooAir,conHeaPID. u_m) annotation (Line(points={{230.2,
            0.12},{400,0.12},{400,40},{440,40},{440,18.8}},
          color={0,0,127}));
    connect(val.port_b, tan1.fluPorVol[8]) annotation (Line(points={{-238,-66},
            {-206,-66},{-206,-32},{-50,-32},{-50,-22.2},{-51,-22.2}}, color={0,
            127,255}));
    connect(wolfBWS1_10_GC.port_a, tan1.fluPorVol[2]) annotation (Line(points={
            {-334.48,-15.24},{-334.48,-24},{-51,-24},{-51,-31.32}}, color={0,
            127,255}));
    connect(domHotWatTan.port_aHea, tan1.fluPorVol1[8]) annotation (Line(points
          ={{216,-286.8},{228,-286.8},{228,-320},{24,-320},{24,-32},{-18.32,-32},
            {-18.32,-21.44}}, color={0,127,255}));
    connect(buiHea.ports_bHeaWat[1], tan1.fluPorVol1[8]) annotation (Line(
          points={{258,-11.1667},{272,-11.1667},{272,-60},{150,-60},{150,-32},{
            -18.32,-32},{-18.32,-21.44}}, color={0,127,255}));
    connect(buiHea.ports_aHeaWat[1], tan1.fluPorVol1[2]) annotation (Line(
          points={{184,-11.1667},{184,-12},{-18.32,-12},{-18.32,-30.56}}, color
          ={0,127,255}));
    connect(domHotWatTan.port_bHea, tan1.fluPorVol1[2]) annotation (Line(points
          ={{146,-286.8},{96,-286.8},{96,-288},{56,-288},{56,-12},{-18.32,-12},
            {-18.32,-30.56}}, color={0,127,255}));
    connect(sinZonFlo.ports[1], buiHea.ports_aHeaWat[2]) annotation (Line(
          points={{212.98,-13.2},{212.98,-8.83333},{184,-8.83333}}, color={0,
            127,255}));
    connect(sinZonFlo.ports[2], buiHea.ports_bHeaWat[2]) annotation (Line(
          points={{213.58,-13.2},{235.64,-13.2},{235.64,-8.83333},{258,-8.83333}},
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
  end SF3;

  model SF4 "Example model of a building with loads provided as time series and
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
          origin={318,-350})));
    Buildings.BoundaryConditions.WeatherData.ReaderTMY3 weaDat(filNam=
          ModelicaServices.ExternalReferences.loadResource(
          "modelica://Buildings/Resources/weatherdata/USA_CA_San.Francisco.Intl.AP.724940_TMY3.mos"))
      annotation (Placement(transformation(extent={{210,110},{230,130}})));
    Modelica.Blocks.Sources.RealExpression realExpression(y=1)
      annotation (Placement(transformation(extent={{-532,-14},{-512,6}})));

    parameter Buildings.DHC.Loads.HotWater.Data.GenericDomesticHotWaterWithHeatExchanger
      datWatHea(VTan=0.1892706, mDom_flow_nominal=6.52944E-06*1000)
      "Data for heat pump water heater with tank"
      annotation (Placement(transformation(extent={{438,-224},{458,-204}})));
    Modelica.Blocks.Sources.Constant conTSetHot(k(
        final unit="K",
        displayUnit="degC") = 313.15)
      "Temperature setpoint for hot water supply to fixture"
      annotation (Placement(transformation(extent={{84,-268},{98,-254}})));
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
    Buildings.Applications.DataCenters.ChillerCooled.Equipment.ElectricHeater eleHea(
      redeclare package Medium = Buildings.Media.Water,
      m_flow_nominal=1000,
      QMax_flow=1000,
      energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
      dp_nominal=6000,
      eta=0.95)
      "Electric heater"
      annotation (Placement(transformation(extent={{-264,-226},{-216,-172}})));
    Modelica.Blocks.Sources.BooleanStep uHea(startTime(displayUnit="min") =
        60000)
      "On/off signal"
      annotation (Placement(transformation(extent={{-528,-206},{-508,-186}})));
    Modelica.Blocks.Sources.Constant conTSetHot1(k(
        final unit="K",
        displayUnit="degC") = 353.15)
      "Temperature setpoint for hot water supply to fixture"
      annotation (Placement(transformation(extent={{-526,-170},{-508,-152}})));
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
      nPorts_bHeaWat=2,
      nPorts_aHeaWat=2)
      "Building with heating only"
      annotation (Placement(transformation(extent={{214,-50},{286,22}})));

    Generators.Digital_Twins.WolfCGB20_GC wolfCGB20_GC
      annotation (Placement(transformation(extent={{-436,-34},{-346,26}})));
    Storage.StratifiedHeatStorage tan1(
      redeclare package Medium = Buildings.Media.Water,
      m_flow_nominal=0.1,
      VTan=775,
      hTan=3,
      dIns=0.3,
      nSeg=10) annotation (Placement(transformation(extent={{-78,-70},{-2,6}})));
    Consumers.DHW domHotWatTan(
      redeclare package MediumDom = Buildings.Media.Water,
      redeclare package MediumHea = Buildings.Media.Water,
      dat=datWatHea)
      annotation (Placement(transformation(extent={{142,-294},{212,-228}})));
    Buildings.ThermalZones.Detailed.Validation.BaseClasses.SingleZoneFloor sinZonFlo(
      redeclare package Medium =
          Modelica.Media.Water.ConstantPropertyLiquidWater,
      use_windPressure=false,
      gai(K=0*[0.4; 0.4; 0.2]))
      "Single-zone floor model"
      annotation (Placement(transformation(extent={{238,-32},{262,-8}})));
    Modelica.Blocks.Math.Gain gaiHea(k=1E6) "Gain for heating"
      annotation (Placement(visible=true,transformation(origin={370,48},
                                                                       extent={{-6,-6},{6,6}},rotation=0)));
    Buildings.Controls.Continuous.LimPID conHeaPID(
      Ti=300,
      k=0.1,
      reverseActing=true,
      strict=true) "Controller for heating"
      annotation (Placement(visible=true, transformation(origin={348,48},extent={{-6,-6},{6,6}},rotation=0)));
    Modelica.Blocks.Sources.Constant TSetHea(k=273.15 + 20)
      "Set-point for heating"
      annotation (Placement(visible=true, transformation(origin={324,48},extent={{-6,-6},{6,6}},rotation=0)));
    Modelica.Blocks.Sources.Constant TSetCoo(k=273.15 + 27)
      "Set-point for cooling"
      annotation (Placement(visible=true, transformation(origin={324,22},extent={{-6,-6},{6,6}},rotation=0)));
    Modelica.Blocks.Math.Gain gaiCoo(k=-1E6) "Gain for cooling"
      annotation (Placement(visible=true,transformation(origin={370,22},
                                                                       extent={{-6,-6},{6,6}},rotation=0)));
    Buildings.Controls.Continuous.LimPID conCooPID(
      Ti=300,
      k=0.1,
      reverseActing=false,
      strict=true) "Controller for cooling"
      annotation (Placement(visible=true, transformation(origin={348,22},extent={{-6,-6},{6,6}},rotation=0)));
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
    connect(uHea.y, eleHea.on) annotation (Line(points={{-507,-196},{-284,-196},
            {-284,-190.9},{-268.8,-190.9}}, color={255,0,255}));
    connect(conTSetHot1.y, eleHea.TSet) annotation (Line(points={{-507.1,-161},
            {-458,-161},{-458,-162},{-408,-162},{-408,-177.4},{-268.8,-177.4}},
                                                        color={0,0,127}));
    connect(eleHea.P, acLoad2.Pow) annotation (Line(points={{-213.6,-215.2},{
            -204,-215.2},{-204,-320},{-210,-320},{-210,-334}}, color={0,0,127}));
    connect(acLoad2.terminal, gri.terminal) annotation (Line(points={{-210,-380},
            {-212,-380},{-212,-410},{82,-410}}, color={0,120,120}));
    connect(buiHea.PPum, acLoad.Pow) annotation (Line(points={{288.4,-6.8},{304,
            -6.8},{304,-312},{318,-312},{318,-328}},color={0,0,127}));
    connect(realExpression.y, wolfCGB20_GC.ControlIn) annotation (Line(points={{-511,-4},
            {-475.15,-4},{-475.15,-3.1},{-439.3,-3.1}},     color={0,0,127}));
    connect(wolfCGB20_GC.weaBus, weaDat.weaBus) annotation (Line(
        points={{-402.4,32.6},{-402.4,120},{230,120}},
        color={255,204,51},
        thickness=0.5));
    connect(wolfCGB20_GC.term_p, gri.terminal) annotation (Line(points={{-388,
            -37},{-388,-410},{82,-410}}, color={0,120,120}));
    connect(domHotWatTan.port_aDom, souCol.ports[2]) annotation (Line(points={{
            142,-241.2},{126,-241.2},{126,-181.3}}, color={0,127,255}));
    connect(domHotWatTan.port_bDom, theMixVal.port_hot) annotation (Line(points
          ={{212,-241.2},{228,-241.2},{228,-204.6},{248,-204.6}}, color={0,127,
            255}));
    connect(domHotWatTan.PEle, acLoad1.Pow) annotation (Line(points={{215.5,
            -261},{250,-261},{250,-326}}, color={0,0,127}));
    connect(conTSetHot.y, domHotWatTan.TDomSet) annotation (Line(points={{98.7,
            -261},{138.5,-261}},              color={0,0,127}));
    connect(eleHea.port_b, tan1.port_a) annotation (Line(points={{-216,-199},{
            -150,-199},{-150,-200},{-82,-200},{-82,20},{-40,20},{-40,6}},
                                                  color={0,127,255}));
    connect(tan1.port_b, eleHea.port_a) annotation (Line(points={{-40,-70},{-40,
            -244},{-264,-244},{-264,-199}}, color={0,127,255}));
    connect(domHotWatTan.port_aHea, tan1.fluPorVol1[8]) annotation (Line(points
          ={{212,-280.8},{224,-280.8},{224,-308},{2,-308},{2,-36},{-26.32,-36},
            {-26.32,-27.44}}, color={0,127,255}));
    connect(buiHea.ports_bHeaWat[1], tan1.fluPorVol1[8]) annotation (Line(
          points={{286,-22.4},{300,-22.4},{300,-64},{2,-64},{2,-36},{-26.32,-36},
            {-26.32,-27.44}}, color={0,127,255}));
    connect(buiHea.ports_aHeaWat[1], tan1.fluPorVol1[2]) annotation (Line(
          points={{214,-22.4},{210,-22.4},{210,-16},{212,-16},{212,-24},{-26.32,
            -24},{-26.32,-36.56}}, color={0,127,255}));
    connect(domHotWatTan.port_bHea, tan1.fluPorVol1[2]) annotation (Line(points
          ={{142,-280.8},{80,-280.8},{80,-280},{18,-280},{18,-24},{-26,-24},{
            -26,-36},{-26.32,-36},{-26.32,-36.56}}, color={0,127,255}));
    connect(wolfCGB20_GC.port_b, tan1.fluPorVol[8]) annotation (Line(points={{
            -349.6,7.4},{-350,7.4},{-350,-44},{-60,-44},{-60,-28.2},{-59,-28.2}},
          color={0,127,255}));
    connect(wolfCGB20_GC.port_a, tan1.fluPorVol[2]) annotation (Line(points={{
            -334,-10.6},{-334,-22},{-60,-22},{-60,-37.32},{-59,-37.32}}, color=
            {0,127,255}));
    connect(conHeaPID.y,gaiHea. u)
      annotation (Line(points={{354.6,48},{362.8,48}},
                                                    color={0,0,127}));
    connect(TSetHea.y,conHeaPID. u_s)
      annotation (Line(points={{330.6,48},{340.8,48}}, color={0,0,127}));
    connect(conCooPID.u_s,TSetCoo. y)
      annotation (Line(points={{340.8,22},{330.6,22}}, color={0,0,127}));
    connect(conCooPID.y,gaiCoo. u)
      annotation (Line(points={{354.6,22},{362.8,22}},
                                                    color={0,0,127}));
    connect(conHeaPID.y,gaiHea. u)
      annotation (Line(points={{354.6,48},{362.8,48}},
                                                    color={0,0,127}));
    connect(gaiCoo.u,conCooPID. y)
      annotation (Line(points={{362.8,22},{354.6,22}},
                                                    color={0,0,127}));
    connect(sinZonFlo.TRooAir,conCooPID. u_m) annotation (Line(points={{260.2,
            -13.88},{348,-13.88},{348,14.8}},                     color={0,0,
            127}));
    connect(sinZonFlo.TRooAir,conHeaPID. u_m) annotation (Line(points={{260.2,
            -13.88},{308,-13.88},{308,34},{348,34},{348,40.8}},
          color={0,0,127}));
    connect(sinZonFlo.weaBus, weaDat.weaBus) annotation (Line(
        points={{242.08,-9.8},{244,-9.8},{244,120},{230,120}},
        color={255,204,51},
        thickness=0.5));
    connect(sinZonFlo.ports[1], buiHea.ports_aHeaWat[2]) annotation (Line(
          points={{242.98,-27.2},{242.98,-20},{214,-20}}, color={0,127,255}));
    connect(sinZonFlo.ports[2], buiHea.ports_bHeaWat[2]) annotation (Line(
          points={{243.58,-27.2},{264.64,-27.2},{264.64,-20},{286,-20}}, color=
            {0,127,255}));
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
        StopTime=1000,
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
      annotation (Placement(transformation(extent={{100,-256},{114,-242}})));
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
    Modelica.Blocks.Sources.BooleanStep uHea(startTime(displayUnit="min") =
        60000)
      "On/off signal"
      annotation (Placement(transformation(extent={{-464,-240},{-444,-220}})));
    Modelica.Blocks.Sources.Ramp y(
      height=1,
      duration=1,
      offset=0) "Control signal"
      annotation (Placement(transformation(extent={{-262,76},{-242,96}})));
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
    Consumers.DHW domHotWatTan(
      redeclare package MediumDom = Buildings.Media.Water,
      redeclare package MediumHea = Buildings.Media.Water,
      dat=datWatHea)
      annotation (Placement(transformation(extent={{138,-282},{208,-216}})));
    Modelica.Blocks.Math.Gain gaiHea(k=1E6) "Gain for heating"
      annotation (Placement(visible=true,transformation(origin={358,12},
                                                                       extent={{-6,-6},{6,6}},rotation=0)));
    Buildings.Controls.Continuous.LimPID conHeaPID(
      Ti=300,
      k=0.1,
      reverseActing=true,
      strict=true) "Controller for heating"
      annotation (Placement(visible=true, transformation(origin={336,12},extent={{-6,-6},{6,6}},rotation=0)));
    Modelica.Blocks.Sources.Constant TSetHea(k=273.15 + 20)
      "Set-point for heating"
      annotation (Placement(visible=true, transformation(origin={312,12},extent={{-6,-6},{6,6}},rotation=0)));
    Modelica.Blocks.Sources.Constant TSetCoo(k=273.15 + 27)
      "Set-point for cooling"
      annotation (Placement(visible=true, transformation(origin={312,-14},
                                                                         extent={{-6,-6},{6,6}},rotation=0)));
    Modelica.Blocks.Math.Gain gaiCoo(k=-1E6) "Gain for cooling"
      annotation (Placement(visible=true,transformation(origin={358,-14},
                                                                       extent={{-6,-6},{6,6}},rotation=0)));
    Buildings.Controls.Continuous.LimPID conCooPID(
      Ti=300,
      k=0.1,
      reverseActing=false,
      strict=true) "Controller for cooling"
      annotation (Placement(visible=true, transformation(origin={336,-14},
                                                                         extent={{-6,-6},{6,6}},rotation=0)));
    Buildings.Fluid.Actuators.Valves.ThreeWayLinear valLin3(
      redeclare package Medium = Buildings.Media.Water,
      l={0.05,0.05},
      m_flow_nominal=2,
      use_inputFilter=false,
      dpValve_nominal=6000,
      energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial)
      "Valve model, linear opening characteristics"
      annotation (Placement(transformation(extent={{96,-110},{116,-90}})));
    Buildings.Fluid.Actuators.Valves.ThreeWayLinear valLin4(
      redeclare package Medium = Buildings.Media.Water,
      l={0.05,0.05},
      m_flow_nominal=2,
      use_inputFilter=false,
      dpValve_nominal=6000,
      energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial)
      "Valve model, linear opening characteristics"
      annotation (Placement(transformation(extent={{72,6},{92,26}})));
    Modelica.Blocks.Sources.Ramp y1(
      height=1,
      duration=1,
      offset=0) "Control signal"
      annotation (Placement(transformation(extent={{130,76},{150,96}})));
    Fluid.Valves.TwoWayEqualPercentage val(
      redeclare package Medium = Media.Water,
      m_flow_nominal=10,
      dpValve_nominal=6000)
      annotation (Placement(transformation(extent={{-186,-52},{-166,-32}})));
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
    connect(buiHea.PPum, acLoad.Pow) annotation (Line(points={{246.467,-16.8},{
            264,-16.8},{264,-168},{320,-168},{320,-328}},
                                color={0,0,127}));
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
    connect(domHotWatTan.port_aDom, souCol.ports[2]) annotation (Line(points={{138,
            -229.2},{126,-229.2},{126,-181.3}},   color={0,127,255}));
    connect(domHotWatTan.port_bDom, theMixVal.port_hot) annotation (Line(points
          ={{208,-229.2},{212,-229.2},{212,-204.6},{248,-204.6}}, color={0,127,
            255}));
    connect(conTSetHot.y, domHotWatTan.TDomSet) annotation (Line(points={{114.7,
            -249},{134.5,-249}},              color={0,0,127}));
    connect(domHotWatTan.PEle, acLoad1.Pow) annotation (Line(points={{211.5,
            -249},{250,-249},{250,-326}}, color={0,0,127}));
    connect(conHeaPID.y,gaiHea. u)
      annotation (Line(points={{342.6,12},{350.8,12}},
                                                    color={0,0,127}));
    connect(TSetHea.y,conHeaPID. u_s)
      annotation (Line(points={{318.6,12},{328.8,12}}, color={0,0,127}));
    connect(conCooPID.u_s,TSetCoo. y)
      annotation (Line(points={{328.8,-14},{318.6,-14}},
                                                       color={0,0,127}));
    connect(conCooPID.y,gaiCoo. u)
      annotation (Line(points={{342.6,-14},{350.8,-14}},
                                                    color={0,0,127}));
    connect(conHeaPID.y,gaiHea. u)
      annotation (Line(points={{342.6,12},{350.8,12}},
                                                    color={0,0,127}));
    connect(gaiCoo.u,conCooPID. y)
      annotation (Line(points={{350.8,-14},{342.6,-14}},
                                                    color={0,0,127}));
    connect(sinZonFlo.TRooAir, conHeaPID.u_m) annotation (Line(points={{216.2,
            -15.88},{216.2,32},{336,32},{336,4.8}}, color={0,0,127}));
    connect(sinZonFlo.TRooAir, conCooPID.u_m) annotation (Line(points={{216.2,
            -15.88},{216.2,32},{300,32},{300,-21.2},{336,-21.2}}, color={0,0,
            127}));
    connect(buiHea.ports_bHeaWat[1], valLin3.port_1) annotation (Line(points={{244,
            -32.4},{256,-32.4},{256,-124},{88,-124},{88,-100},{96,-100}}, color
          ={0,127,255}));
    connect(domHotWatTan.port_aHea, valLin3.port_3) annotation (Line(points={{
            208,-268.8},{216,-268.8},{216,-268},{222,-268},{222,-292},{58,-292},
            {58,-114},{106,-114},{106,-110}}, color={0,127,255}));
    connect(valLin4.port_2, buiHea.ports_aHeaWat[1]) annotation (Line(points={{92,16},
            {160,16},{160,-32.4},{170,-32.4}},     color={0,127,255}));
    connect(valLin4.port_3, domHotWatTan.port_bHea) annotation (Line(points={{82,6},{
            82,-156},{80,-156},{80,-268.8},{138,-268.8}},
                                   color={0,127,255}));
    connect(y1.y, valLin4.y)
      annotation (Line(points={{151,86},{82,86},{82,28}},color={0,0,127}));
    connect(y1.y, valLin3.y) annotation (Line(points={{151,86},{82,86},{82,40},
            {100,40},{100,-76},{106,-76},{106,-88}},
                        color={0,0,127}));
    connect(y.y, val.y) annotation (Line(points={{-241,86},{-176,86},{-176,-30}},
          color={0,0,127}));
    connect(wolfCGB50_GC.port_b, val.port_a) annotation (Line(points={{-388.74,
            37.44},{-388,37.44},{-388,-42},{-186,-42}}, color={0,127,255}));
    connect(valLin3.port_2, tan.fluPorVol1[8]) annotation (Line(points={{116,
            -100},{124,-100},{124,-4},{7.68,-4},{7.68,8.56}}, color={0,127,255}));
    connect(valLin4.port_1, tan.fluPorVol1[2]) annotation (Line(points={{72,16},
            {8,16},{8,8},{7.68,8},{7.68,-0.56}}, color={0,127,255}));
    connect(neoTower5_GC.port_a, tan.fluPorVol[2]) annotation (Line(points={{
            -328.56,-259.96},{-332,-259.96},{-332,14},{-25,14},{-25,-1.32}},
          color={0,127,255}));
    connect(wolfCGB50_GC.port_a, tan.fluPorVol[2]) annotation (Line(points={{
            -380.16,13.02},{-380.16,4},{-332,4},{-332,14},{-25,14},{-25,-1.32}},
          color={0,127,255}));
    connect(val.port_b, tan.fluPorVol[8]) annotation (Line(points={{-166,-42},{
            -126,-42},{-126,-6},{-25,-6},{-25,7.8}}, color={0,127,255}));
    connect(neoTower5_GC.port_b, tan.fluPorVol[8]) annotation (Line(points={{
            -335.38,-241.36},{-126,-241.36},{-126,-6},{-25,-6},{-25,7.8}},
          color={0,127,255}));
    connect(sinZonFlo.ports[1], buiHea.ports_aHeaWat[2]) annotation (Line(
          points={{198.98,-29.2},{198.98,-30},{170,-30}}, color={0,127,255}));
    connect(sinZonFlo.ports[2], buiHea.ports_bHeaWat[2]) annotation (Line(
          points={{199.58,-29.2},{222.64,-29.2},{222.64,-30},{244,-30}}, color=
            {0,127,255}));
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
        StopTime=900,
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
      Buildings.BoundaryConditions.WeatherData.ReaderTMY3 weaDat(filNam=
            ModelicaServices.ExternalReferences.loadResource(
            "modelica://Buildings/Resources/weatherdata/USA_CA_San.Francisco.Intl.AP.724940_TMY3.mos"))
        annotation (Placement(transformation(extent={{156,90},{176,110}})));
      Modelica.Blocks.Sources.RealExpression realExpression(y=1)
        annotation (Placement(transformation(extent={{-488,-66},{-468,-46}})));
      Buildings.DHC.Loads.BaseClasses.BuildingTimeSeries buiHea(
        have_chiWat=false,
        filNam=
            "modelica://Buildings/Resources/Data/DHC/Loads/Examples/SwissResidential_20190916.mos",
        nPorts_aChiWat=1,
        nPorts_bChiWat=1,
        nPorts_bHeaWat=1,
        nPorts_aHeaWat=1)
        "Building with heating only"
        annotation (Placement(transformation(extent={{32,-58},{62,-14}})));

      Generators.Digital_Twins.NeoTower2_GC neoTower2_GC
        annotation (Placement(transformation(extent={{-448,-82},{-372,-30}})));
      Storage.StratifiedHeatStorage tan(
        redeclare package Medium = Buildings.Media.Water,
        m_flow_nominal=2,
        VTan(displayUnit="l") = 0.775,
        hTan=3,
        dIns=0.3,
        nSeg=10,
        T_start=333.15)
        annotation (Placement(transformation(extent={{-220,-70},{-144,6}})));
      Modelica.Fluid.Sensors.MassFlowRate massFlowRate(redeclare package Medium =
            Modelica.Media.Water.ConstantPropertyLiquidWater)
        annotation (Placement(transformation(extent={{-296,-98},{-276,-78}})));
      Modelica.Blocks.Logical.LessThreshold lessThreshold(threshold=340)
        annotation (Placement(transformation(extent={{-542,-22},{-522,-2}})));
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
      connect(buiHea.PPum, acLoad.Pow) annotation (Line(points={{63,-31.6},{88,
              -31.6},{88,-112}}, color={0,0,127}));
      connect(neoTower2_GC.weaBus, weaDat.weaBus) annotation (Line(
          points={{-400.975,-12.0364},{-400.975,100},{176,100}},
          color={255,204,51},
          thickness=0.5));
      connect(neoTower2_GC.CHPModulation, realExpression.y) annotation (Line(
            points={{-434.938,-41.1091},{-460,-41.1091},{-460,-56},{-467,-56}},
                                     color={0,0,127}));
      connect(neoTower2_GC.term_p, gri.terminal) annotation (Line(points={{-401.45,
              -71.1273},{-401.45,-232},{16,-232},{16,-220}},         color={0,
              120,120}));
      connect(neoTower2_GC.port_b, tan.fluPorVol[3]) annotation (Line(points={{-387.2,
              -47.0182},{-232,-47.0182},{-232,-35.8},{-201,-35.8}},
            color={0,127,255}));
      connect(tan.fluPorVol1[3], buiHea.ports_bHeaWat[1]) annotation (Line(
            points={{-168.32,-35.04},{-168.32,-28},{20,-28},{20,-72},{76,-72},{
              76,-40.4},{62,-40.4}}, color={0,127,255}));
      connect(massFlowRate.port_b, tan.fluPorVol[8]) annotation (Line(points={{
              -276,-88},{-240,-88},{-240,-44},{-232,-44},{-232,-28.2},{-201,
              -28.2}}, color={0,127,255}));
      connect(massFlowRate.port_a, neoTower2_GC.port_a) annotation (Line(points
            ={{-296,-88},{-360,-88},{-360,-61.6727},{-386.25,-61.6727}}, color=
              {0,127,255}));
      connect(lessThreshold.y, neoTower2_GC.CHPOn) annotation (Line(points={{-521,
              -12},{-464,-12},{-464,-25.5091},{-434.938,-25.5091}}, color={255,
              0,255}));
      connect(buiHea.ports_aHeaWat[1], tan.port_a) annotation (Line(points={{32,
              -40.4},{28,-40.4},{28,20},{-182,20},{-182,6}}, color={0,127,255}));
      connect(tan.Tempside, lessThreshold.u) annotation (Line(points={{-151.6,
              -32},{-132,-32},{-132,24},{-388,24},{-388,-20},{-412,-20},{-412,8},
              {-552,8},{-552,-12},{-544,-12}}, color={0,0,127}));
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
          StopTime=100,
          Tolerance=1e-06,
          __Dymola_Algorithm="Dassl"),
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
      Buildings.BoundaryConditions.WeatherData.ReaderTMY3 weaDat(filNam=
            ModelicaServices.ExternalReferences.loadResource(
            "modelica://Buildings/Resources/weatherdata/USA_CA_San.Francisco.Intl.AP.724940_TMY3.mos"))
        annotation (Placement(transformation(extent={{156,90},{176,110}})));
      Modelica.Blocks.Sources.RealExpression realExpression(y=1)
        annotation (Placement(transformation(extent={{-448,-32},{-428,-12}})));
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

      Storage.StratifiedHeatStorage tan1(
        redeclare package Medium = Buildings.Media.Water,
        m_flow_nominal=2,
        VTan(displayUnit="l") = 0.775,
        hTan=3,
        dIns=0.3,
        nSeg=10,
        T_start=333.15)
        annotation (Placement(transformation(extent={{-194,-78},{-118,-2}})));
      Generators.Digital_Twins.WolfCGB14_GC wolfCGB14_GC
        annotation (Placement(transformation(extent={{-408,-64},{-276,12}})));
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
      connect(buiHea.PPum, acLoad.Pow) annotation (Line(points={{237.467,-39.2},
              {276,-39.2},{276,-286}},
                                  color={0,0,127}));
      connect(wolfCGB14_GC.weaBus, weaDat.weaBus) annotation (Line(
          points={{-355.2,5.78182},{-355.2,84},{184,84},{184,100},{176,100}},
          color={255,204,51},
          thickness=0.5));
      connect(wolfCGB14_GC.ControlIn, realExpression.y) annotation (Line(points
            ={{-394.453,-36.0182},{-420,-36.0182},{-420,-22},{-427,-22}}, color
            ={0,0,127}));
      connect(wolfCGB14_GC.port_a, tan1.fluPorVol[8]) annotation (Line(points={
              {-288.505,-34.9818},{-208,-34.9818},{-208,-36.2},{-175,-36.2}},
            color={0,127,255}));
      connect(wolfCGB14_GC.port_b, tan1.fluPorVol[3]) annotation (Line(points={
              {-297.537,-6.65455},{-216,-6.65455},{-216,-43.8},{-175,-43.8}},
            color={0,127,255}));
      connect(tan1.fluPorVol1[8], buiHea.ports_aHeaWat[1]) annotation (Line(
            points={{-142.32,-35.44},{180,-35.44},{180,-50.8},{192,-50.8}},
            color={0,127,255}));
      connect(tan1.fluPorVol1[3], buiHea.ports_bHeaWat[1]) annotation (Line(
            points={{-142.32,-43.04},{-142.32,-52},{-102,-52},{-102,-88},{248,
              -88},{248,-50.8},{236,-50.8}}, color={0,127,255}));
      connect(wolfCGB14_GC.term_p, gri.terminal) annotation (Line(points={{
              -338.526,-74.3636},{-338.526,-392},{202,-392},{202,-380}}, color=
              {0,120,120}));
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
          StopTime=1000,
          Tolerance=1e-06,
          __Dymola_Algorithm="Dassl"),
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
      Buildings.BoundaryConditions.WeatherData.ReaderTMY3 weaDat(filNam=
            ModelicaServices.ExternalReferences.loadResource(
            "modelica://Buildings/Resources/weatherdata/USA_CA_San.Francisco.Intl.AP.724940_TMY3.mos"))
        annotation (Placement(transformation(extent={{208,114},{228,134}})));
      Modelica.Blocks.Sources.RealExpression realExpression(y=1)
        annotation (Placement(transformation(extent={{-442,6},{-422,26}})));
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

      Storage.StratifiedHeatStorage tan1(
        redeclare package Medium = Buildings.Media.Water,
        m_flow_nominal=2,
        VTan(displayUnit="l") = 0.775,
        hTan=3,
        dIns=0.3,
        nSeg=10,
        T_start=333.15)
        annotation (Placement(transformation(extent={{-130,-64},{-54,12}})));
      Generators.Digital_Twins.NeoTower2_GC neoTower2_GC annotation (Placement(
            transformation(extent={{-484,-196},{-368,-116}})));
      Generators.Digital_Twins.WolfCGB14_GC wolfCGB14_GC
        annotation (Placement(transformation(extent={{-632,14},{-478,104}})));
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
      connect(y.y, valLin1.y) annotation (Line(points={{-277,78},{-234,78},{-234,
              -4}}, color={0,0,127}));
      connect(y.y, valLin.y) annotation (Line(points={{-277,78},{-236,78},{-236,8},
              {-162,8},{-162,-126}}, color={0,0,127}));
      connect(buiHea.PPum, acLoad.Pow) annotation (Line(points={{163.467,-21.2},
              {276,-21.2},{276,-286}},
                                  color={0,0,127}));
      connect(neoTower2_GC.port_b, valLin.port_3) annotation (Line(points={{
              -391.2,-142.182},{-180,-142.182},{-180,-148},{-162,-148}}, color=
              {0,127,255}));
      connect(neoTower2_GC.port_a, valLin1.port_3) annotation (Line(points={{
              -389.75,-164.727},{-308,-164.727},{-308,-36},{-234,-36},{-234,-26}},
            color={0,127,255}));
      connect(wolfCGB14_GC.port_a, valLin1.port_1) annotation (Line(points={{
              -492.589,48.3636},{-412,48.3636},{-412,52},{-264,52},{-264,-16},{
              -244,-16}}, color={0,127,255}));
      connect(wolfCGB14_GC.port_b, valLin.port_1) annotation (Line(points={{
              -503.126,81.9091},{-312,81.9091},{-312,-132},{-316,-132},{-316,
              -138},{-172,-138}}, color={0,127,255}));
      connect(booleanExpression.y, neoTower2_GC.CHPOn) annotation (Line(points=
              {{-517,-112},{-496,-112},{-496,-108},{-466,-108},{-466,-112},{-464.062,
              -112},{-464.062,-109.091}}, color={255,0,255}));
      connect(realExpression1.y, neoTower2_GC.CHPModulation) annotation (Line(
            points={{-517,-156},{-500,-156},{-500,-133.091},{-464.062,-133.091}},
            color={0,0,127}));
      connect(wolfCGB14_GC.weaBus, weaDat.weaBus) annotation (Line(
          points={{-570.4,96.6364},{-570.4,110},{-570,110},{-570,124},{228,124}},
          color={255,204,51},
          thickness=0.5));

      connect(neoTower2_GC.weaBus, weaDat.weaBus) annotation (Line(
          points={{-412.225,-88.3636},{-414,-88.3636},{-414,-76},{-520,-76},{
              -520,124},{228,124}},
          color={255,204,51},
          thickness=0.5));
      connect(neoTower2_GC.term_p, gri.terminal) annotation (Line(points={{
              -412.95,-179.273},{-412.95,-392},{202,-392},{202,-380}}, color={0,
              120,120}));
      connect(buiHea.ports_aHeaWat[1], tan1.fluPorVol1[8]) annotation (Line(
            points={{118,-32.8},{-44,-32.8},{-44,-21.44},{-78.32,-21.44}},
            color={0,127,255}));
      connect(buiHea.ports_bHeaWat[1], tan1.fluPorVol1[3]) annotation (Line(
            points={{162,-32.8},{176,-32.8},{176,-68},{-36,-68},{-36,-32},{-44,
              -32},{-44,-29.04},{-78.32,-29.04}}, color={0,127,255}));
      connect(tan1.fluPorVol[8], valLin1.port_2) annotation (Line(points={{-111,
              -22.2},{-111,-28},{-212,-28},{-212,-16},{-224,-16}}, color={0,127,
              255}));
      connect(tan1.fluPorVol[3], valLin.port_2) annotation (Line(points={{-111,
              -29.8},{-110,-29.8},{-110,-32},{-108,-32},{-108,-36},{-142,-36},{
              -142,-138},{-152,-138}}, color={0,127,255}));
      connect(wolfCGB14_GC.term_p, gri.terminal) annotation (Line(points={{
              -550.947,1.72727},{-550.947,-384},{-412,-384},{-412,-392},{202,
              -392},{202,-380}}, color={0,120,120}));
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
          StopTime=1000,
          Tolerance=1e-06,
          __Dymola_Algorithm="Dassl"),
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

      Storage.StratifiedHeatStorage tan1(
        redeclare package Medium = Buildings.Media.Water,
        m_flow_nominal=2,
        VTan(displayUnit="l") = 0.775,
        hTan=3,
        dIns=0.3,
        nSeg=10,
        T_start=333.15)
        annotation (Placement(transformation(extent={{-160,-72},{-84,4}})));
      Fluid.Pumps.Test_Pump_controler test_Pump_controler
        annotation (Placement(transformation(extent={{-378,-116},{-358,-96}})));
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
      connect(weaDat.weaBus, solCol.weaBus) annotation (Line(
          points={{176,100},{112,100},{112,98},{-420,98},{-420,60.92}},
          color={255,204,51},
          thickness=0.5));
      connect(buiHea.PPum, acLoad.Pow) annotation (Line(points={{79.4,-49.2},{
              84,-49.2},{84,-96},{90,-96},{90,-112}},
                                 color={0,0,127}));
      connect(solCol.port_a, test_Pump_controler.port_a) annotation (Line(
            points={{-420,35},{-432,35},{-432,-105.8},{-377.6,-105.8}}, color={
              0,127,255}));
      connect(test_Pump_controler.term_p, gri.terminal) annotation (Line(points
            ={{-362.4,-119.4},{-362.4,-224},{14,-224},{14,-206}}, color={0,120,
              120}));
      connect(y.y, test_Pump_controler.SetInSignal) annotation (Line(points={{
              -345,-44},{-336,-44},{-336,-84},{-376.2,-84},{-376.2,-95}}, color
            ={0,0,127}));
      connect(solCol.port_b, tan1.fluPorVol[8]) annotation (Line(points={{-360,
              35},{-172,35},{-172,-30.2},{-141,-30.2}}, color={0,127,255}));
      connect(tan1.fluPorVol[3], test_Pump_controler.port_b) annotation (Line(
            points={{-141,-37.8},{-328,-37.8},{-328,-105.8},{-357.6,-105.8}},
            color={0,127,255}));
      connect(tan1.fluPorVol1[8], buiHea.ports_aHeaWat[1]) annotation (Line(
            points={{-108.32,-29.44},{24,-29.44},{24,-60.8},{36,-60.8}}, color=
              {0,127,255}));
      connect(tan1.fluPorVol1[3], buiHea.ports_bHeaWat[1]) annotation (Line(
            points={{-108.32,-37.04},{-108.32,-28},{24,-28},{24,-16},{92,-16},{
              92,-60.8},{78,-60.8}}, color={0,127,255}));
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
          StopTime=1000,
          Tolerance=1e-06,
          __Dymola_Algorithm="Dassl"),
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
      Buildings.BoundaryConditions.WeatherData.ReaderTMY3 weaDat(filNam=
            ModelicaServices.ExternalReferences.loadResource(
            "modelica://Buildings/Resources/weatherdata/USA_CA_San.Francisco.Intl.AP.724940_TMY3.mos"))
        annotation (Placement(transformation(extent={{208,114},{228,134}})));
      Modelica.Blocks.Sources.RealExpression realExpression(y=1)
        annotation (Placement(transformation(extent={{-662,-12},{-642,8}})));
      Modelica.Blocks.Sources.RealExpression realExpression1(y=1)
        annotation (Placement(transformation(extent={{-660,-250},{-640,-230}})));
      Modelica.Blocks.Sources.BooleanExpression booleanExpression(y=true)
        annotation (Placement(transformation(extent={{-660,-206},{-640,-186}})));
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

      Modelica.Blocks.Sources.Pulse y1(
        offset=0.25,
        startTime=0,
        amplitude=0.5,
        period=15*60) "Input signal"
                     annotation (Placement(transformation(extent={{-268,-320},{
                -248,-300}})));
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

      Storage.StratifiedHeatStorage tan2(
        redeclare package Medium = Buildings.Media.Water,
        m_flow_nominal=2,
        VTan(displayUnit="l") = 0.775,
        hTan=3,
        dIns=0.3,
        nSeg=10,
        T_start=333.15)
        annotation (Placement(transformation(extent={{78,-290},{154,-214}})));
      Storage.StratifiedHeatStorage tan3(
        redeclare package Medium = Buildings.Media.Water,
        m_flow_nominal=2,
        VTan(displayUnit="l") = 0.775,
        hTan=3,
        dIns=0.3,
        nSeg=10,
        T_start=333.15)
        annotation (Placement(transformation(extent={{-16,-60},{60,16}})));
      Generators.Digital_Twins.WolfCGB14_GC wolfCGB14_GC
        annotation (Placement(transformation(extent={{-614,-22},{-496,46}})));
      Generators.Digital_Twins.NeoTower2_GC neoTower2_GC annotation (Placement(
            transformation(extent={{-586,-286},{-478,-212}})));
      Fluid.Pumps.Test_Pump_controler test_Pump_controler
        annotation (Placement(transformation(extent={{-246,-354},{-226,-334}})));
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
      connect(y.y, valLin1.y) annotation (Line(points={{-277,78},{-234,78},{-234,
              -4}}, color={0,0,127}));
      connect(y.y, valLin.y) annotation (Line(points={{-277,78},{-236,78},{-236,8},
              {-162,8},{-162,-126}}, color={0,0,127}));
      connect(solCol.weaBus, weaDat.weaBus) annotation (Line(
          points={{-302,-207.08},{-328,-207.08},{-328,124},{228,124}},
          color={255,204,51},
          thickness=0.5));
      connect(buiHea.PPum, acLoad.Pow) annotation (Line(points={{227.333,-26.4},
              {276,-26.4},{276,-286}},
                                  color={0,0,127}));
      connect(tan2.fluPorVol[8], solCol.port_b) annotation (Line(points={{97,
              -248.2},{60,-248.2},{60,-233},{-242,-233}}, color={0,127,255}));
      connect(test_Pump_controler.port_a, solCol.port_a) annotation (Line(
            points={{-245.6,-343.8},{-320,-343.8},{-320,-233},{-302,-233}},
            color={0,127,255}));
      connect(test_Pump_controler.port_b, tan2.fluPorVol[3]) annotation (Line(
            points={{-225.6,-343.8},{64,-343.8},{64,-255.8},{97,-255.8}}, color
            ={0,127,255}));
      connect(test_Pump_controler.term_p, gri.terminal) annotation (Line(points
            ={{-230.4,-357.4},{-230.4,-392},{202,-392},{202,-380}}, color={0,
              120,120}));
      connect(y1.y, test_Pump_controler.SetInSignal) annotation (Line(points={{
              -247,-310},{-247,-324},{-244.2,-324},{-244.2,-333}}, color={0,0,
              127}));
      connect(neoTower2_GC.port_b, valLin.port_3) annotation (Line(points={{
              -499.6,-236.218},{-376,-236.218},{-376,-148},{-162,-148}}, color=
              {0,127,255}));
      connect(wolfCGB14_GC.port_b, valLin.port_1) annotation (Line(points={{
              -515.253,29.3091},{-456,29.3091},{-456,-76},{-352,-76},{-352,-108},
              {-316,-108},{-316,-138},{-172,-138}}, color={0,127,255}));
      connect(wolfCGB14_GC.port_a, valLin1.port_1) annotation (Line(points={{
              -507.179,3.96364},{-444,3.96364},{-444,-24},{-264,-24},{-264,-16},
              {-244,-16}}, color={0,127,255}));
      connect(neoTower2_GC.port_a, valLin1.port_3) annotation (Line(points={{
              -498.25,-257.073},{-348,-257.073},{-348,-136},{-234,-136},{-234,
              -26}}, color={0,127,255}));
      connect(wolfCGB14_GC.term_p, gri.terminal) annotation (Line(points={{
              -551.895,-31.2727},{-551.895,-176},{-604,-176},{-604,-384},{-232,
              -384},{-232,-392},{202,-392},{202,-380}}, color={0,120,120}));
      connect(wolfCGB14_GC.weaBus, weaDat.weaBus) annotation (Line(
          points={{-566.8,40.4364},{-566.8,124},{228,124}},
          color={255,204,51},
          thickness=0.5));
      connect(realExpression.y, wolfCGB14_GC.ControlIn) annotation (Line(points
            ={{-641,-2},{-641,-4},{-628,-4},{-628,3.03636},{-601.889,3.03636}},
            color={0,0,127}));
      connect(booleanExpression.y, neoTower2_GC.CHPOn) annotation (Line(points=
              {{-639,-196},{-608,-196},{-608,-172},{-567.438,-172},{-567.438,-205.609}},
            color={255,0,255}));
      connect(neoTower2_GC.CHPModulation, realExpression1.y) annotation (Line(
            points={{-567.438,-227.809},{-628,-227.809},{-628,-240},{-639,-240}},
            color={0,0,127}));
      connect(neoTower2_GC.weaBus, weaDat.weaBus) annotation (Line(
          points={{-519.175,-186.436},{-522,-186.436},{-522,-186},{-526,-186},{
              -526,124},{228,124}},
          color={255,204,51},
          thickness=0.5));
      connect(valLin1.port_2, tan3.fluPorVol[8]) annotation (Line(points={{-224,
              -16},{-212,-16},{-212,48},{-28,48},{-28,-18.2},{3,-18.2}}, color=
              {0,127,255}));
      connect(valLin.port_2, tan3.fluPorVol[3]) annotation (Line(points={{-152,
              -138},{-152,-140},{-144,-140},{-144,-48},{-156,-48},{-156,48},{
              -28,48},{-28,-25.8},{3,-25.8}}, color={0,127,255}));
      connect(tan3.fluPorVol1[8], buiHea.ports_aHeaWat[1]) annotation (Line(
            points={{35.68,-17.44},{176,-17.44},{176,-37.6},{186,-37.6}}, color
            ={0,127,255}));
      connect(tan3.fluPorVol1[3], buiHea.ports_bHeaWat[1]) annotation (Line(
            points={{35.68,-25.04},{35.68,-16},{176,-16},{176,-72},{240,-72},{
              240,-37.6},{226,-37.6}}, color={0,127,255}));
      connect(neoTower2_GC.term_p, gri.terminal) annotation (Line(points={{
              -519.85,-270.527},{-519.85,-384},{-232,-384},{-232,-392},{202,
              -392},{202,-380}}, color={0,120,120}));
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
          StopTime=1000,
          Tolerance=1e-06,
          __Dymola_Algorithm="Dassl"),
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
      Buildings.BoundaryConditions.WeatherData.ReaderTMY3 weaDat(filNam=
            ModelicaServices.ExternalReferences.loadResource(
            "modelica://Buildings/Resources/weatherdata/USA_CA_San.Francisco.Intl.AP.724940_TMY3.mos"))
        annotation (Placement(transformation(extent={{156,90},{176,110}})));
      Modelica.Blocks.Sources.RealExpression realExpression1(y=1)
        annotation (Placement(transformation(extent={{-544,-10},{-524,10}})));
      Modelica.Blocks.Sources.RealExpression realExpression5(y=1)
        annotation (Placement(transformation(extent={{-546,-40},{-526,-20}})));
      Modelica.Blocks.Sources.BooleanExpression booleanExpression(y=true)
        annotation (Placement(transformation(extent={{-542,30},{-522,50}})));
      Modelica.Blocks.Sources.BooleanExpression booleanExpression1(y=true)
        annotation (Placement(transformation(extent={{-540,60},{-520,80}})));
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

      Storage.StratifiedHeatStorage tan1(
        redeclare package Medium = Buildings.Media.Water,
        m_flow_nominal=2,
        VTan(displayUnit="l") = 0.775,
        hTan=3,
        dIns=0.3,
        nSeg=10,
        T_start=333.15)
        annotation (Placement(transformation(extent={{-208,-10},{-132,66}})));
      Generators.Digital_Twins.WolfCHA10_GC wolfCHA10_GC
        annotation (Placement(transformation(extent={{-484,-2},{-414,68}})));
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
      connect(buiHea.PPum, acLoad.Pow) annotation (Line(points={{53.2667,-1.2},
              {88,-1.2},{88,-112}},
                                color={0,0,127}));
      connect(wolfCHA10_GC.port_b, tan1.fluPorVol[3]) annotation (Line(points={
              {-395.1,46.3},{-395.1,40},{-220,40},{-220,24.2},{-189,24.2}},
            color={0,127,255}));
      connect(wolfCHA10_GC.port_a, tan1.fluPorVol[8]) annotation (Line(points={
              {-400.7,30.9},{-400.7,31.8},{-189,31.8}}, color={0,127,255}));
      connect(wolfCHA10_GC.term_p, gri.terminal) annotation (Line(points={{
              -429.4,-13.9},{-429.4,-224},{14,-224},{14,-206}}, color={0,120,
              120}));
      connect(wolfCHA10_GC.weaBus, weaDat.weaBus) annotation (Line(
          points={{-428.7,73.6},{-428.7,100},{176,100}},
          color={255,204,51},
          thickness=0.5));
      connect(booleanExpression1.y, wolfCHA10_GC.HPOn) annotation (Line(points=
              {{-519,70},{-500,70},{-500,51.55},{-480.15,51.55}}, color={255,0,
              255}));
      connect(booleanExpression.y, wolfCHA10_GC.HPMode) annotation (Line(points
            ={{-521,40},{-500.575,40},{-500.575,38.25},{-480.15,38.25}}, color=
              {255,0,255}));
      connect(realExpression1.y, wolfCHA10_GC.HPModulation) annotation (Line(
            points={{-523,0},{-496,0},{-496,24.25},{-480.15,24.25}}, color={0,0,
              127}));
      connect(realExpression5.y, wolfCHA10_GC.HPAuxModulation) annotation (Line(
            points={{-525,-30},{-525,-32},{-500,-32},{-500,10.95},{-480.15,
              10.95}}, color={0,0,127}));
      connect(tan1.fluPorVol1[8], buiHea.ports_aHeaWat[1]) annotation (Line(
            points={{-156.32,32.56},{4,32.56},{4,-12.8},{14,-12.8}}, color={0,
              127,255}));
      connect(buiHea.ports_bHeaWat[1], tan1.fluPorVol1[3]) annotation (Line(
            points={{52,-12.8},{64,-12.8},{64,-48},{-108,-48},{-108,24.96},{
              -156.32,24.96}}, color={0,127,255}));
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
      Buildings.DHC.Loads.BaseClasses.BuildingTimeSeries buiCoo(
        have_heaWat=false,
        filNam=
            "modelica://Buildings/Resources/Data/DHC/Loads/Examples/SwissResidential_20190916.mos",
        nPorts_aChiWat=1,
        nPorts_bChiWat=1)
        "Building wint cooling only"
        annotation (Placement(transformation(extent={{-2,-30},{30,18}})));

      Storage.StratifiedHeatStorage tan1(
        redeclare package Medium = Buildings.Media.Water,
        m_flow_nominal=2,
        VTan(displayUnit="l") = 0.775,
        hTan=3,
        dIns=0.3,
        nSeg=10,
        T_start=333.15)
        annotation (Placement(transformation(extent={{-232,-38},{-156,38}})));
      Generators.Digital_Twins.WolfCHA10_GC wolfCHA10_GC
        annotation (Placement(transformation(extent={{-474,-40},{-412,22}})));
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
      connect(buiCoo.PPum, acLoad.Pow) annotation (Line(points={{31.0667,-1.2},
              {88,-1.2},{88,-112}},   color={0,0,127}));
      connect(tan1.fluPorVol1[8], buiCoo.ports_aChiWat[1]) annotation (Line(
            points={{-180.32,4.56},{-16,4.56},{-16,-20.4},{-2,-20.4}}, color={0,
              127,255}));
      connect(buiCoo.ports_bChiWat[1], tan1.fluPorVol1[3]) annotation (Line(
            points={{30,-20.4},{44,-20.4},{44,-44},{-136,-44},{-136,-3.04},{
              -180.32,-3.04}}, color={0,127,255}));
      connect(tan1.fluPorVol[3], wolfCHA10_GC.port_b) annotation (Line(points={
              {-213,-3.8},{-213,2.78},{-395.26,2.78}}, color={0,127,255}));
      connect(wolfCHA10_GC.port_a, tan1.fluPorVol[8]) annotation (Line(points={
              {-400.22,-10.86},{-252,-10.86},{-252,3.8},{-213,3.8}}, color={0,
              127,255}));
      connect(wolfCHA10_GC.HPOn, booleanExpression1.y) annotation (Line(points=
              {{-470.59,7.43},{-512,7.43},{-512,70},{-519,70}}, color={255,0,
              255}));
      connect(wolfCHA10_GC.HPMode, booleanExpression.y) annotation (Line(points
            ={{-470.59,-4.35},{-512,-4.35},{-512,4},{-521,4},{-521,40}}, color=
              {255,0,255}));
      connect(wolfCHA10_GC.HPModulation, realExpression1.y) annotation (Line(
            points={{-470.59,-16.75},{-516,-16.75},{-516,0},{-523,0}}, color={0,
              0,127}));
      connect(wolfCHA10_GC.HPAuxModulation, realExpression5.y) annotation (Line(
            points={{-470.59,-28.53},{-476,-28.53},{-476,-32},{-523,-32}},
            color={0,0,127}));
      connect(wolfCHA10_GC.term_p, gri.terminal) annotation (Line(points={{
              -425.64,-50.54},{-425.64,-224},{14,-224},{14,-206}}, color={0,120,
              120}));
      connect(wolfCHA10_GC.weaBus, weaDat.weaBus) annotation (Line(
          points={{-425.02,26.96},{-425.02,100},{176,100}},
          color={255,204,51},
          thickness=0.5));
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
