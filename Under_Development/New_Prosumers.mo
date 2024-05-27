within ProsNet.Under_Development;
package New_Prosumers

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
    Modelica.Fluid.Interfaces.FluidPort_a port_a1
      "Fluid connector a (positive design flow direction is from port_a to port_b)"
      annotation (Placement(transformation(extent={{26,172},{46,192}})));
    Modelica.Fluid.Interfaces.FluidPort_b port_b1
      "Fluid connector b (positive design flow direction is from port_a to port_b)"
      annotation (Placement(transformation(extent={{-60,170},{-40,190}})));
    Buildings.ThermalZones.Detailed.Validation.BaseClasses.SingleZoneFloor sinZonFlo(
      redeclare package Medium =
          Modelica.Media.Water.ConstantPropertyLiquidWater,
      use_windPressure=false,
      gai(K=0*[0.4; 0.4; 0.2]))
      "Single-zone floor model"
      annotation (Placement(transformation(extent={{198,-68},{222,-44}})));
    Modelica.Blocks.Math.Gain gaiHea(k=1E6) "Gain for heating"
      annotation (Placement(visible=true,transformation(origin={360,28},
                                                                       extent={{-6,-6},{6,6}},rotation=0)));
    Buildings.Controls.Continuous.LimPID conHeaPID(
      Ti=300,
      k=0.1,
      reverseActing=true,
      strict=true) "Controller for heating"
      annotation (Placement(visible=true, transformation(origin={338,28},extent={{-6,-6},{6,6}},rotation=0)));
    Modelica.Blocks.Sources.Constant TSetHea(k=273.15 + 20)
      "Set-point for heating"
      annotation (Placement(visible=true, transformation(origin={314,28},extent={{-6,-6},{6,6}},rotation=0)));
    Modelica.Blocks.Sources.Constant TSetCoo(k=273.15 + 27)
      "Set-point for cooling"
      annotation (Placement(visible=true, transformation(origin={314,2}, extent={{-6,-6},{6,6}},rotation=0)));
    Modelica.Blocks.Math.Gain gaiCoo(k=-1E6) "Gain for cooling"
      annotation (Placement(visible=true,transformation(origin={360,2},extent={{-6,-6},{6,6}},rotation=0)));
    Buildings.Controls.Continuous.LimPID conCooPID(
      Ti=300,
      k=0.1,
      reverseActing=false,
      strict=true) "Controller for cooling"
      annotation (Placement(visible=true, transformation(origin={338,2}, extent={{-6,-6},{6,6}},rotation=0)));
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
            -59.4667},{144,-59.4667},{144,40},{36,40},{36,-8},{-8,-8},{-8,-22},
            {-8.32,-22},{-8.32,-22.56}},
          color={0,127,255}));
    connect(valLin2.port_2, tan.fluPorVol1[8]) annotation (Line(points={{110,
            -124},{36,-124},{36,-24},{-8,-24},{-8,-14},{-8.32,-14},{-8.32,
            -13.44}},                                    color={0,127,255}));
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
    connect(tan.port_a, port_a1) annotation (Line(points={{-22,20},{8,20},{8,
            182},{36,182}}, color={0,127,255}));
    connect(tan.port_b, port_b1) annotation (Line(points={{-22,-56},{-36,-56},{
            -36,180},{-50,180}}, color={0,127,255}));
    connect(sinZonFlo.ports[1], buiHea.ports_aHeaWat[2]) annotation (Line(
          points={{202.98,-63.2},{202.98,-57.3333},{174,-57.3333}}, color={0,
            127,255}));
    connect(sinZonFlo.ports[2], buiHea.ports_bHeaWat[2]) annotation (Line(
          points={{203.58,-63.2},{226.64,-63.2},{226.64,-57.3333},{248,-57.3333}},
          color={0,127,255}));
    connect(sinZonFlo.weaBus, weaDat.weaBus) annotation (Line(
        points={{202.08,-45.8},{202.08,120},{230,120}},
        color={255,204,51},
        thickness=0.5));
    connect(conHeaPID.y,gaiHea. u)
      annotation (Line(points={{344.6,28},{352.8,28}},
                                                    color={0,0,127}));
    connect(TSetHea.y,conHeaPID. u_s)
      annotation (Line(points={{320.6,28},{330.8,28}}, color={0,0,127}));
    connect(conCooPID.u_s,TSetCoo. y)
      annotation (Line(points={{330.8,2},{320.6,2}},   color={0,0,127}));
    connect(conCooPID.y,gaiCoo. u)
      annotation (Line(points={{344.6,2},{352.8,2}},color={0,0,127}));
    connect(conHeaPID.y,gaiHea. u)
      annotation (Line(points={{344.6,28},{352.8,28}},
                                                    color={0,0,127}));
    connect(gaiCoo.u,conCooPID. y)
      annotation (Line(points={{352.8,2},{344.6,2}},color={0,0,127}));
    connect(sinZonFlo.TRooAir,conHeaPID. u_m) annotation (Line(points={{220.2,
            -49.88},{220.2,48},{338,48},{338,20.8}},
          color={0,0,127}));
    connect(sinZonFlo.TRooAir, conCooPID.u_m) annotation (Line(points={{220.2,
            -49.88},{220.2,48},{286,48},{286,-12},{338,-12},{338,-5.2}}, color=
            {0,0,127}));
    annotation (
      Icon(
        coordinateSystem(
          preserveAspectRatio=false, extent={{-460,-500},{380,120}}), graphics
          ={Rectangle(extent={{-210,-2},{204,-432}}, lineColor={28,108,200}),
            Line(points={{-208,0},{-2,118},{202,0}}, color={28,108,200})}),
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

  package validation_SF1
    model Test_heat_transfer_station_production
      ProsNet.Under_Development.new_prosumer_models.heat_transfer_station
                                                B1(
        n=0.5,
        redeclare ProsNet.Fluid.Pumps.Data.Pumps.IMP.NMTSmart25_120to180
          feedinPer,
        R_ins_transferpipe=100000)                        annotation (Placement(
            transformation(
            extent={{20,-18},{-20,18}},
            rotation=0,
            origin={-18,-54})));
      Modelica.Blocks.Math.Add add annotation (Placement(transformation(
            extent={{-5,-5},{5,5}},
            rotation=-90,
            origin={-1,41})));
      Modelica.Blocks.Sources.RealExpression realExpression(y=273.15) annotation (
          Placement(transformation(
            extent={{-5,-6},{5,6}},
            rotation=270,
            origin={22,55})));
      ProsNet.Under_Development.Controller_PID_based.auxiliary.TimeTable_noInterp
                                                        power_set1(table=[0,4;
            3600,-4; 7200,10; 10800,-10; 14400,-6; 18000,6; 21600,10; 25200,10])
                                                                   annotation (Placement(
            transformation(
            extent={{-10,-10},{10,10}},
            rotation=-90,
            origin={-36,76})));
      ProsNet.Under_Development.Controller_PID_based.auxiliary.TimeTable_noInterp
                                                        temp_sec_in1(table=[0,
            55; 3600,30; 7200,55; 10800,30; 14400,30; 18000,55; 21600,55; 25200,
            55])                                                     annotation (
          Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=-90,
            origin={6,76})));
      ProsNet.Under_Development.Controller_PID_based.PID_Q_T_weighted_crossover
                                                      Ctrl1(
        alpha_prim_prod=1,
        alpha_sec_prod=0,
        alpha_prim_cons=0,
        alpha_sec_cons=1)
        annotation (Placement(transformation(extent={{-28,-16},{-4,18}})));
      Modelica.Blocks.Sources.Ramp ramp(
        height=1,
        duration=600,
        offset=0,
        startTime=200)
        annotation (Placement(transformation(extent={{-64,-186},{-44,-166}})));
      Modelica.Fluid.Vessels.ClosedVolume volume(
        T_start=318.15,
        use_portsData=false,
        V=1,
        nPorts=2,
        redeclare final package Medium = ProsNet.Media.Water)
        annotation (Placement(transformation(extent={{30,-178},{50,-198}})));
      ProsNet.Fluid.Valves.TwoWayEqualPercentage
                                         valve_for_test(
        m_flow_nominal=30.074213*0.001/60,
        redeclare final package Medium = ProsNet.Media.Water,
        final CvData=ProsNet.Fluid.Types.CvTypes.Kv,
        final Kv=2.5,
        final use_inputFilter=true,
        final riseTime=5,
        final init=Modelica.Blocks.Types.Init.InitialOutput,
        final y_start=0,
        final l=2e-3) annotation (Placement(transformation(
            extent={{10,10},{-10,-10}},
            rotation=-90,
            origin={64,-158})));
      ProsNet.Fluid.Sources.Boundary_pT
                                bou(nPorts=1, redeclare final package Medium =
            ProsNet.Media.Water)
        annotation (Placement(transformation(extent={{134,-152},{114,-132}})));
    equation
      connect(add.u1,realExpression. y) annotation (Line(points={{2,47},{2,49.5},
              {22,49.5}},                color={0,0,127}));
      connect(temp_sec_in1.y,add. u2) annotation (Line(points={{6,65},{6,47},{
              -4,47}},         color={0,0,127}));
      connect(power_set1.y,Ctrl1. Q_dot_set) annotation (Line(points={{-36,65},
              {-36,28},{-22,28},{-22,18.8}},
                                    color={0,0,127}));
      connect(Ctrl1.states,B1. states)
        annotation (Line(points={{-28,0},{-48,0},{-48,-54},{-38,-54}},
                                                                     color={0,0,127}));
      connect(Ctrl1.contr_vars_real,B1. contr_vars_real)
        annotation (Line(points={{-4,0},{12,0},{12,-54},{2.2,-54}},    color={0,0,127}));
      connect(add.y,Ctrl1. T_sec_in_is) annotation (Line(points={{-1,35.5},{-2,
              35.5},{-2,28},{-10,28},{-10,19}},
                             color={0,0,127}));
      connect(valve_for_test.port_a,volume. ports[1])
        annotation (Line(points={{64,-168},{64,-170},{39,-170},{39,-178}},
                                                              color={0,127,255}));
      connect(ramp.y,valve_for_test. y) annotation (Line(points={{-43,-176},{26,
              -176},{26,-158},{52,-158}},
                              color={0,0,127}));
      connect(bou.ports[1],valve_for_test. port_b)
        annotation (Line(points={{114,-142},{64,-142},{64,-148}},
                                                          color={0,127,255}));
      connect(B1.hot_prim, valve_for_test.port_b) annotation (Line(points={{-4,
              -72.2},{-4,-110},{64,-110},{64,-148}}, color={0,127,255}));
      connect(B1.cold_prim, volume.ports[2]) annotation (Line(points={{-32,-72},
              {-32,-162},{41,-162},{41,-178}}, color={0,127,255}));
      annotation (Diagram(graphics={                               Rectangle(extent={{-62,92},
                  {28,-82}},      lineColor={28,108,200})}));
    end Test_heat_transfer_station_production;

    model Test_heat_transfer_station_Consumption
      ProsNet.Under_Development.new_prosumer_models.heat_transfer_station
                                                B2(
        n=0.5,
        redeclare ProsNet.Fluid.Pumps.Data.Pumps.IMP.NMTSmart25_120to180
          feedinPer,
        R_ins_transferpipe=100000)                        annotation (Placement(
            transformation(
            extent={{20,-18},{-20,18}},
            rotation=0,
            origin={-8,-54})));
      Modelica.Blocks.Sources.RealExpression realExpression1(y=273.15) annotation (
          Placement(transformation(
            extent={{-5,-6},{5,6}},
            rotation=270,
            origin={24,53})));
      Modelica.Blocks.Math.Add add1 annotation (Placement(transformation(
            extent={{-5,-5},{5,5}},
            rotation=-90,
            origin={1,39})));
      ProsNet.Under_Development.Controller_PID_based.auxiliary.TimeTable_noInterp
                                                        power_set2(table=[0,-10;
            3600,10; 7200,-4; 10800,4; 14400,10; 18000,-10; 21600,-4; 25200,-4])
                                                                   annotation (Placement(
            transformation(
            extent={{-10,-10},{10,10}},
            rotation=-90,
            origin={-26,76})));
      ProsNet.Under_Development.Controller_PID_based.auxiliary.TimeTable_noInterp
                                                        temp_sec_in2(table=[0,
            30; 3600,55; 7200,30; 10800,55; 14400,55; 18000,30; 21600,30; 25200,
            30])                                                     annotation (
          Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=-90,
            origin={16,76})));
      ProsNet.Under_Development.Controller_PID_based.PID_Q_T_weighted_crossover
                                                      Ctrl2(
        alpha_prim_prod=1,
        alpha_sec_prod=0,
        alpha_prim_cons=0,
        alpha_sec_cons=1) annotation (Placement(transformation(extent={{-18,-14},
                {6,20}})));
      Modelica.Blocks.Sources.Ramp ramp(
        height=1,
        duration=600,
        offset=0,
        startTime=200)
        annotation (Placement(transformation(extent={{-90,-178},{-70,-158}})));
      Modelica.Fluid.Vessels.ClosedVolume volume(
        T_start=338.15,
        use_portsData=false,
        V=1,
        nPorts=2,
        redeclare final package Medium = ProsNet.Media.Water)
        annotation (Placement(transformation(extent={{4,-176},{24,-196}})));
      ProsNet.Fluid.Sources.Boundary_pT
                                bou(redeclare final package Medium =
            ProsNet.Media.Water, nPorts=1)
        annotation (Placement(transformation(extent={{108,-142},{88,-122}})));
      ProsNet.Fluid.Pumps.SpeedControlled_y
                                    pump_prim_prod(
        redeclare final package Medium = ProsNet.Media.Water,
        final energyDynamics=Modelica.Fluid.Types.Dynamics.DynamicFreeInitial,
        final tau=1,
        redeclare final ProsNet.Fluid.Pumps.Data.Pumps.IMP.NMTPlus152025to40
          per,
        final use_inputFilter=true,
        final riseTime=5,
        final init=Modelica.Blocks.Types.Init.InitialOutput,
        final y_start=0)                annotation (Placement(transformation(
            extent={{10,10},{-10,-10}},
            rotation=-90,
            origin={42,-144})));
    equation
      connect(realExpression1.y,add1. u1) annotation (Line(points={{24,47.5},{
              24,45},{4,45}},             color={0,0,127}));
      connect(temp_sec_in2.y,add1. u2)
        annotation (Line(points={{16,65},{16,64},{-2,64},{-2,45}},
                                                              color={0,0,127}));
      connect(power_set2.y,Ctrl2. Q_dot_set) annotation (Line(points={{-26,65},
              {-26,30},{-12,30},{-12,20.8}},
                                   color={0,0,127}));
      connect(add1.y,Ctrl2. T_sec_in_is)
        annotation (Line(points={{1,33.5},{1,28},{0,28},{0,21}},
                                                               color={0,0,127}));
      connect(Ctrl2.contr_vars_real,B2. contr_vars_real)
        annotation (Line(points={{6,2},{22,2},{22,-54},{12.2,-54}},color={0,0,127}));
      connect(Ctrl2.states,B2. states)
        annotation (Line(points={{-18,2},{-38,2},{-38,-54},{-28,-54}},
                                                                 color={0,0,127}));
      connect(volume.ports[1],pump_prim_prod. port_a) annotation (Line(points={{13,-176},
              {13,-160},{42,-160},{42,-154}},      color={0,127,255}));
      connect(ramp.y,pump_prim_prod. y) annotation (Line(points={{-69,-168},{20,
              -168},{20,-144},{30,-144}},
                              color={0,0,127}));
      connect(B2.cold_prim, volume.ports[2]) annotation (Line(points={{-22,-72},
              {-22,-166},{15,-166},{15,-176}}, color={0,127,255}));
      connect(pump_prim_prod.port_b, B2.hot_prim) annotation (Line(points={{42,
              -134},{42,-78},{6,-78},{6,-72.2}}, color={0,127,255}));
      connect(pump_prim_prod.port_b, bou.ports[1]) annotation (Line(points={{42,
              -134},{42,-132},{88,-132}}, color={0,127,255}));
      annotation (Diagram(graphics={                               Rectangle(extent={{-52,92},
                  {38,-82}},      lineColor={28,108,200})}));
    end Test_heat_transfer_station_Consumption;
  end validation_SF1;
end New_Prosumers;
