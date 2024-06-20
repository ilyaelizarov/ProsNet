within ProsNet.Under_Development;
package New_Prosumers

  model SF1_indep_Newcontrol "Example model of a building with loads provided as time series and
  connected to an ETS for cooling"
    extends Modelica.Icons.Example;
    package Medium=ProsNet.Fluid.Building_Fluid.Utili.Media.Water
      "Medium model";
    Modelica.Blocks.Sources.RealExpression realExpression(y=1)
      annotation (Placement(transformation(extent={{-580,38},{-560,58}})));
    Modelica.Blocks.Sources.RealExpression realExpression1(y=1)
      annotation (Placement(transformation(extent={{-550,-154},{-530,-134}})));
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
          origin={449,-175})));

    Modelica.Blocks.Sources.Constant conTSetMix(k(
        final unit="K",
        displayUnit="degC") = 308.15)
      "Temperature setpoint for mixed water supply to fixture"
      annotation (Placement(transformation(extent={{-8,-8},{8,8}},
          rotation=180,
          origin={448,-148})));

    Modelica.Blocks.Sources.Ramp y1(
      height=1,
      duration=1,
      offset=0) "Control signal"
      annotation (Placement(transformation(extent={{92,18},{112,38}})));
    Modelica.Blocks.Sources.Constant conTSetHot(k(
        final unit="K",
        displayUnit="degC") = 313.15)
      "Temperature setpoint for hot water supply to fixture"
      annotation (Placement(transformation(extent={{8,-8},{-8,8}},
          rotation=0,
          origin={48,-300})));
    Fluid.Pumps.FlowControlled_m_flow pump(m_flow_nominal=10, addPowerToMedium=
          true)
      annotation (Placement(transformation(extent={{-118,-298},{-74,-264}})));
    Components.Generators.Digital_Twins.WolfCGB14_GC wolfCGB14_GC1
      annotation (Placement(transformation(extent={{-524,2},{-432,56}})));
    Components.Generators.Digital_Twins.NeoTower2_GC neoTower2_GC1
      annotation (Placement(transformation(extent={{-528,-194},{-436,-130}})));
    Fluid.Sources.Boundary_pT souCol(redeclare package Medium =
          ProsNet.Media.Water, nPorts=1)
      annotation (Placement(transformation(extent={{170,-196},{190,-176}})));

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
    Components.Generators.Laboratory_Models.ASHRAE93 solCol(
      redeclare package Medium = ProsNet.Fluid.Building_Fluid.Utili.Media.Water,
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
      annotation (Placement(transformation(extent={{-424,-362},{-332,-272}})));

    Components.Consumers.ThermostaticMixingValve theMixVal(mMix_flow_nominal=1.2*
          datWatHea.mDom_flow_nominal)
      annotation (Placement(transformation(extent={{254,-242},{294,-206}})));
    Components.Consumers.StorageTankWithExternalHeatExchanger domHotWatTan(
      redeclare package MediumDom =
          ProsNet.Fluid.Building_Fluid.Utili.Media.Water,
      redeclare package MediumHea =
          ProsNet.Fluid.Building_Fluid.Utili.Media.Water,
      dat=datWatHea)
      annotation (Placement(transformation(extent={{76,-336},{146,-264}})));
    Components.Electrical.Inductive acLoad2(linearized=false, mode=ProsNet.Fluid.Building_Fluid.Utili.Electrical.Types.Load.VariableZ_P_input)
      annotation (Placement(transformation(
          extent={{-22,-17},{22,17}},
          rotation=90,
          origin={-53,-446})));
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
      annotation (Placement(transformation(extent={{440,-228},{460,-208}})));
    Controls.BooleanToReal booToRea(realTrue=2.5, realFalse=0)
      annotation (Placement(transformation(extent={{-132,-244},{-112,-224}})));
    Components.Valves.ThreeWayLinear val3(
    redeclare package Medium = ProsNet.Fluid.Building_Fluid.Utili.Media.Water,
      energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
      use_inputFilter=false,
      m_flow_nominal=2,
      dpValve_nominal=1,
      l={0.05,0.05})
      annotation (Placement(transformation(extent={{-230,-30},{-210,-10}})));
    Components.Valves.ThreeWayLinear val4(
      redeclare package Medium = Fluid.Building_Fluid.Utili.Media.Water,
      energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
      use_inputFilter=false,
      m_flow_nominal=2,
      dpValve_nominal=1,
      l={0.05,0.05})
      annotation (Placement(transformation(extent={{-162,-150},{-142,-130}})));
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
      VTan(displayUnit="l") = 0.775,
      hTan=3,
      dIns=0.3,
      nSeg=10,
      T_start=333.15)
      annotation (Placement(transformation(extent={{-108,-60},{10,58}})));
    Components.Consumers.SingleZoneFloor sinZonFlo(redeclare package Medium =
          Modelica.Media.Water.ConstantPropertyLiquidWater, use_windPressure=
          false)
      annotation (Placement(transformation(extent={{204,-26},{244,14}})));
    Components.Consumers.BuildingTimeSeries buiHea(
      have_chiWat=false,
      filNam="modelica://ProsNet/SwissResidential_20190916.mos",
      nPorts_aHeaWat=2,
      nPorts_bHeaWat=2)
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
    Controls.GeneratorControlPackage.ControlChargingStorage.HeatingStorageControl
      heatingStorageControl(TStartCHP=338, TStopCHP=343)
      annotation (Placement(transformation(extent={{-744,4},{-668,80}})));
    Modelica.Blocks.Sources.BooleanExpression booleanExpression(y=true)
      annotation (Placement(transformation(extent={{-896,8},{-836,60}})));
    Modelica.Fluid.Interfaces.FluidPort_a port_a2
      "Fluid connector a (positive design flow direction is from port_a to port_b)"
      annotation (Placement(transformation(extent={{18,202},{38,222}})));
    Modelica.Fluid.Interfaces.FluidPort_b port_b1
      "Fluid connector b (positive design flow direction is from port_a to port_b)"
      annotation (Placement(transformation(extent={{62,202},{82,222}})));
  equation
    connect(wolfCGB14_GC1.ControlIn, realExpression.y) annotation (Line(points={{
            -516.737,42.2545},{-516.737,48},{-559,48}}, color={0,0,127}));
    connect(neoTower2_GC1.CHPModulation, realExpression1.y) annotation (Line(
          points={{-512.188,-143.673},{-519.594,-143.673},{-519.594,-144},{-529,
            -144}}, color={0,0,127}));

    connect(pump.port_a, solCol.port_b) annotation (Line(points={{-118,-281},{-312,
            -281},{-312,-317},{-332,-317}},                         color={0,127,255}));
    connect(souCol.ports[1], theMixVal.port_col) annotation (Line(points={{190,-186},
            {200,-186},{200,-238.4},{254,-238.4}}, color={0,127,255}));
    connect(theMixVal.TMixSet, conTSetMix.y) annotation (Line(points={{252,-220.4},
            {244,-220.4},{244,-196},{468,-196},{468,-132},{432,-132},{432,-148},{439.2,
            -148}}, color={0,0,127}));
    connect(sch.y[1], theMixVal.yMixSet) annotation (Line(points={{439.1,-175},{240,
            -175},{240,-209.6},{252,-209.6}}, color={0,0,127}));
    connect(conTSetHot.y, domHotWatTan.TDomSet)
      annotation (Line(points={{39.2,-300},{72.5,-300}}, color={0,0,127}));
    connect(theMixVal.port_hot, domHotWatTan.port_bDom) annotation (Line(points={{
            254,-231.2},{160,-231.2},{160,-278.4},{146,-278.4}}, color={0,127,255}));
    connect(solCol.port_a, domHotWatTan.port_bHea) annotation (Line(points={{-424,
            -317},{-440,-317},{-440,-376},{60,-376},{60,-374},{64,-374},{64,-321.6},
            {76,-321.6}}, color={0,127,255}));
    connect(domHotWatTan.port_aHea, pump.port_b) annotation (Line(points={{146,-321.6},
            {152,-321.6},{152,-260},{-60,-260},{-60,-281},{-74,-281}}, color={0,127,
            255}));
    connect(acLoad2.Pow, pump.P) annotation (Line(points={{-53,-424},{-53,-265.7},
            {-71.8,-265.7}}, color={0,0,127}));
    connect(acLoad1.Pow, domHotWatTan.PEle) annotation (Line(points={{168,-422},{168,
            -300},{149.5,-300}}, color={0,0,127}));
    connect(pv.terminal, conv.terminal_p) annotation (Line(points={{476,-324},{464,
            -324},{464,-608},{16,-608}}, color={0,0,255}));
    connect(bat.terminal, conv.terminal_p) annotation (Line(points={{-470,-610},{-488,
            -610},{-488,-664},{28,-664},{28,-608},{16,-608}}, color={0,0,255}));
    connect(con.SOC, bat.SOC) annotation (Line(points={{-232,-628},{-348,-628},{-348,
            -584.8},{-369.2,-584.8}}, color={0,0,127}));
    connect(con.y, bat.P) annotation (Line(points={{-197,-628},{-188,-628},{-188,-540},
            {-422,-540},{-422,-568}}, color={0,0,127}));
    connect(wolfCGB14_GC1.term_p, gri.terminal) annotation (Line(points={{-422.8,9.85455},
            {-436,9.85455},{-436,10},{-448,10},{-448,-528},{-446,-528},{-446,-530},
            {50,-530}}, color={0,120,120}));
    connect(acLoad.terminal, gri.terminal) annotation (Line(points={{302,-466},{302,
            -532},{100,-532},{100,-530},{50,-530}}, color={0,120,120}));
    connect(acLoad1.terminal, gri.terminal) annotation (Line(points={{168,-470},{168,
            -532},{110,-532},{110,-530},{50,-530}}, color={0,120,120}));
    connect(conv.terminal_n, gri.terminal) annotation (Line(points={{-72,-608},{-84,
            -608},{-84,-530},{50,-530}}, color={0,120,120}));
    connect(acLoad2.terminal, gri.terminal) annotation (Line(points={{-53,-468},{-53,
            -530},{50,-530}}, color={0,120,120}));
    connect(neoTower2_GC1.term_p, gri.terminal) annotation (Line(points={{-439.45,
            -192.255},{-448,-192.255},{-448,-434},{-446,-434},{-446,-530},{50,
            -530}},
          color={0,120,120}));
    connect(booToRea.u, domHotWatTan.charge) annotation (Line(points={{-134,-234},
            {-142,-234},{-142,-348},{153,-348},{153,-332.4}}, color={255,0,255}));
    connect(booToRea.y, pump.m_flow_in) annotation (Line(points={{-110,-234},{-98,
            -234},{-98,-260.6},{-96,-260.6}}, color={0,0,127}));
    connect(wolfCGB14_GC1.port_a, val3.port_1) annotation (Line(points={{
            -440.716,22.6182},{-240,22.6182},{-240,-20},{-230,-20}},
                                                            color={0,127,255}));
    connect(val3.port_3, neoTower2_GC1.port_a) annotation (Line(points={{-220,
            -30},{-220,-168.982},{-453.25,-168.982}},
                                                 color={0,127,255}));
    connect(val3.y, y.y)
      annotation (Line(points={{-220,-8},{-220,66},{-269,66}}, color={0,0,127}));
    connect(val4.port_3, neoTower2_GC1.port_b) annotation (Line(points={{-152,
            -150},{-152,-160},{-428,-160},{-428,-128},{-454.4,-128},{-454.4,
            -150.945}}, color={0,127,255}));
    connect(val4.port_1, wolfCGB14_GC1.port_b) annotation (Line(points={{-162,
            -140},{-414,-140},{-414,68},{-447.011,68},{-447.011,42.7455}}, color=
            {0,127,255}));
    connect(val4.y, y.y) annotation (Line(points={{-152,-128},{-152,66},{-269,66}},
          color={0,0,127}));
    connect(val1.port_3, domHotWatTan.port_aDom) annotation (Line(points={{156,
            -132},{156,-128},{36,-128},{36,-278},{76,-278},{76,-278.4}},   color=
            {0,127,255}));
    connect(val1.y, y1.y) annotation (Line(points={{156,-154},{156,-164},{120,-164},
            {120,28},{113,28}},                    color={0,0,127}));
    connect(pv.weaBus, weaDat.weaBus) annotation (Line(
        points={{520,-282.6},{520,130},{-48,130}},
        color={255,204,51},
        thickness=0.5));
    connect(weaDat.weaBus, solCol.weaBus) annotation (Line(
        points={{-48,130},{-248,130},{-248,-248},{-424,-248},{-424,-273.8}},
        color={255,204,51},
        thickness=0.5));
    connect(wolfCGB14_GC1.weaBus, weaDat.weaBus) annotation (Line(
        points={{-559.832,39.3091},{-559.832,24},{-600,24},{-600,130},{-48,130}},
        color={255,204,51},
        thickness=0.5));

    connect(weaDat.weaBus, neoTower2_GC1.weaBus) annotation (Line(
        points={{-48,130},{-600,130},{-600,-120.691},{-570.55,-120.691}},
        color={255,204,51},
        thickness=0.5));
    connect(val3.port_2, tan.fluPorVol[1]) annotation (Line(points={{-210,-20},{
            -210,14},{-80,14},{-80,-11.62},{-78.5,-11.62}},   color={0,127,255}));
    connect(val4.port_2, tan.fluPorVol[10]) annotation (Line(points={{-142,-140},
            {-132,-140},{-132,-8},{-78,-8},{-78,9.62},{-78.5,9.62}},   color={0,
            127,255}));
    connect(val1.port_2, tan.fluPorVol1[10]) annotation (Line(points={{146,-142},
            {88,-142},{88,-16},{2,-16},{2,10},{1.74,10},{1.74,10.8}},
                                                                color={0,127,255}));
    connect(sinZonFlo.weaBus, weaDat.weaBus) annotation (Line(
        points={{210.8,11},{132,11},{132,130},{-48,130}},
        color={255,204,51},
        thickness=0.5));
    connect(tan.fluPorVol1[1], buiHea.ports_aHeaWat[1]) annotation (Line(points={
            {1.74,-10.44},{1.74,12},{68,12},{68,-4},{124,-4},{124,-5.06667},{172,
            -5.06667}}, color={0,127,255}));
    connect(val1.port_1, buiHea.ports_bHeaWat[1]) annotation (Line(points={{166,
            -142},{292,-142},{292,-5.06667},{278,-5.06667}}, color={0,127,255}));
    connect(buiHea.PPum, acLoad.Pow) annotation (Line(points={{281.533,19.2},{
            294,19.2},{294,20},{308,20},{308,-404},{302,-404},{302,-418}},
                                                                       color={0,0,
            127}));
    connect(sinZonFlo.ports[2], buiHea.ports_bHeaWat[2]) annotation (Line(points=
            {{213.3,-18},{213.3,-4},{272,-4},{272,-1.33333},{278,-1.33333}},
          color={0,127,255}));
    connect(sinZonFlo.ports[1], buiHea.ports_aHeaWat[2]) annotation (Line(points=
            {{212.3,-18},{212.3,-12},{212,-12},{212,-4},{174,-4},{174,-1.33333},{
            172,-1.33333}}, color={0,127,255}));
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
    connect(heatingStorageControl.CHPON, wolfCGB14_GC1.CBOn1) annotation (Line(
          points={{-673.32,42},{-588,42},{-588,20},{-540,20},{-540,29.9818},{
            -519.158,29.9818}},
                       color={255,0,255}));
    connect(heatingStorageControl.CHPON, neoTower2_GC1.CHPOn) annotation (Line(
          points={{-673.32,42},{-588,42},{-588,20},{-540,20},{-540,-104},{
            -512.188,-104},{-512.188,-124.473}},
                                        color={255,0,255}));
    connect(tan.T1, heatingStorageControl.TopTlayer) annotation (Line(points={{11.18,
            55.64},{11.18,72},{-260,72},{-260,84},{-656,84},{-656,92},{-776,92},{-776,
            54.16},{-754.64,54.16}}, color={0,0,127}));
    connect(tan.T10, heatingStorageControl.LowTlayer) annotation (Line(points={
            {31.24,-52.92},{48,-52.92},{48,-76},{-560,-76},{-560,-128},{-776,-128},
            {-776,12.36},{-754.64,12.36}}, color={0,0,127}));
    connect(booleanExpression.y, heatingStorageControl.UseChp) annotation (Line(
          points={{-833,34},{-793.82,34},{-793.82,35.16},{-754.64,35.16}},
          color={255,0,255}));
    connect(port_a2, tan.fluPorVol1[2]) annotation (Line(points={{28,212},{28,
            16},{1.74,16},{1.74,-8.08}}, color={0,127,255}));
    connect(port_b1, tan.fluPorVol1[2]) annotation (Line(points={{72,212},{72,
            -12},{1.74,-12},{1.74,-8.08}}, color={0,127,255}));
                                                        annotation (Line(points={{-754.64,38.96},{-786.32,38.96},{-786.32,40},{-818,
            40}}, color={255,0,255}),
      Icon(
        coordinateSystem(
          preserveAspectRatio=false, extent={{-460,-500},{380,120}}), graphics
          ={
          Rectangle(extent={{-158,2},{152,-350}}, lineColor={28,108,200}),
          Line(points={{-156,12},{2,122}}, color={28,108,200}),
          Line(points={{4,120},{152,6}}, color={28,108,200})}),
      Diagram(
          coordinateSystem(
          preserveAspectRatio=false, extent={{-460,-500},{380,120}})),
      __Dymola_Commands(
        file="modelica://Buildings/Resources/Scripts/Dymola/DHC/Loads/Heating/Examples/BuildingTimeSeriesWithETS.mos" "Simulate and plot"),
      experiment(
        StopTime=19500,
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
  end SF1_indep_Newcontrol;

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
