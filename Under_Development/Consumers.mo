within ProsNet.Under_Development;
package Consumers
  model BuildingTimeSeries
    "Building model with heating and/or cooling loads provided as time series"
    extends Buildings.DHC.Loads.BaseClasses.PartialBuilding(
      redeclare package Medium=Buildings.Media.Water,
      have_heaWat=true,
      have_chiWat=true,
      final have_fan=false,
      final have_pum=true,
      final have_eleHea=false,
      final have_eleCoo=false,
      final have_weaBus=false);
    replaceable package Medium2=Buildings.Media.Air
      constrainedby Modelica.Media.Interfaces.PartialMedium
      "Load side medium";
    parameter Boolean have_hotWat = false
      "Set to true if SHW load is included in the time series"
      annotation (Evaluate=true, Dialog(group="Configuration"));
    parameter String filNam
      "File name with thermal loads as time series";
    parameter Real facMulHea(min=0)=QHea_flow_nominal /
      (QHea_flow_nominal_ref * abs(T_aLoaHea_nominal - T_aHeaWat_nominal) /
       abs(T_aLoaHea_nominal_ref - T_aHeaWat_nominal_ref) *
       mLoaHea_flow_nominal / mLoaHea_flow_nominal_ref)
      "Heating terminal unit multiplier factor"
      annotation(Dialog(enable=have_heaWat, group="Scaling", tab="Advanced"));
    parameter Real facMulCoo(min=0)=QCoo_flow_nominal /
      (QCoo_flow_nominal_ref * abs(h_aLoaCoo_nominal - hSat_nominal) /
       abs(h_aLoaCoo_nominal_ref - hSat_nominal_ref) *
       mLoaCoo_flow_nominal / mLoaCoo_flow_nominal_ref)
      "Cooling terminal unit scaling factor"
      annotation(Dialog(enable=have_chiWat, group="Scaling", tab="Advanced"));
    parameter Modelica.Units.SI.Temperature T_aHeaWat_nominal=323.15
      "Heating water inlet temperature at nominal conditions"
      annotation (Dialog(group="Nominal condition", enable=have_heaWat));
    parameter Modelica.Units.SI.Temperature T_bHeaWat_nominal(
      min=273.15,
      displayUnit="degC") = T_aHeaWat_nominal - 10
      "Heating water outlet temperature at nominal conditions"
      annotation (Dialog(group="Nominal condition", enable=have_heaWat));
    parameter Modelica.Units.SI.Temperature T_aChiWat_nominal=280.15
      "Chilled water inlet temperature at nominal conditions "
      annotation (Dialog(group="Nominal condition", enable=have_chiWat));
    parameter Modelica.Units.SI.Temperature T_bChiWat_nominal(
      min=273.15,
      displayUnit="degC") = T_aChiWat_nominal + 5
      "Chilled water outlet temperature at nominal conditions"
      annotation (Dialog(group="Nominal condition", enable=have_chiWat));
    parameter Modelica.Units.SI.Temperature T_aLoaHea_nominal=293.15
      "Load side inlet temperature at nominal conditions in heating mode"
      annotation (Dialog(group="Nominal condition", tab="Advanced"));
    parameter Modelica.Units.SI.Temperature T_aLoaCoo_nominal=298.15
      "Load side inlet temperature at nominal conditions in cooling mode"
      annotation (Dialog(group="Nominal condition", tab="Advanced", enable=have_chiWat));
    parameter Modelica.Units.SI.MassFraction w_aLoaCoo_nominal=0.01
      "Load side inlet humidity ratio at nominal conditions in cooling mode"
      annotation (Dialog(group="Nominal condition", tab="Advanced", enable=have_chiWat));
    parameter Modelica.Units.SI.MassFlowRate mLoaHea_flow_nominal(min=Modelica.Constants.eps)=0.5
      "Load side mass flow rate at nominal conditions in heating mode (single unit)"
      annotation (Dialog(group="Nominal condition", tab="Advanced", enable=have_heaWat));
    parameter Modelica.Units.SI.MassFlowRate mLoaCoo_flow_nominal(min=Modelica.Constants.eps)=
      mLoaHea_flow_nominal
      "Load side mass flow rate at nominal conditions in cooling mode (single unit)"
      annotation (Dialog(group="Nominal condition", tab="Advanced", enable=have_chiWat));

    parameter Modelica.Units.SI.Temperature T_aHeaWat_nominal_ref=323.15
      "Heating water inlet temperature at nominal conditions of reference terminal unit"
      annotation(Dialog(enable=have_heaWat, group="Reference terminal unit performance", tab="Advanced"));
    parameter Modelica.Units.SI.Temperature T_aLoaHea_nominal_ref=293.15
      "Load side inlet temperature at nominal conditions in heating mode of reference terminal unit"
      annotation(Dialog(enable=have_heaWat, group="Reference terminal unit performance", tab="Advanced"));
    parameter Modelica.Units.SI.MassFlowRate mLoaHea_flow_nominal_ref(min=Modelica.Constants.eps) = 0.5
      "Load side mass flow rate at nominal conditions in heating mode of reference terminal unit"
      annotation(Dialog(enable=have_heaWat, group="Reference terminal unit performance", tab="Advanced"));
    parameter Modelica.Units.SI.HeatFlowRate QHea_flow_nominal_ref(min=Modelica.Constants.eps) = 4.5E3
      "Heat flow at nominal conditions in heating mode of reference terminal unit"
      annotation(Dialog(enable=have_heaWat, group="Reference terminal unit performance", tab="Advanced"));

    parameter Modelica.Units.SI.Temperature T_aChiWat_nominal_ref=279.15
      "Chilled water inlet temperature at nominal conditions of reference terminal unit"
      annotation(Dialog(enable=have_chiWat, group="Reference terminal unit performance", tab="Advanced"));
    parameter Modelica.Units.SI.Temperature T_aLoaCoo_nominal_ref=298.15
      "Load side inlet temperature at nominal conditions in cooling mode of reference terminal unit"
      annotation(Dialog(enable=have_chiWat, group="Reference terminal unit performance", tab="Advanced"));
    parameter Modelica.Units.SI.MassFraction w_aLoaCoo_nominal_ref=0.01
      "Load side inlet humidity ratio at nominal conditions in cooling mode of reference terminal unit"
      annotation(Dialog(enable=have_chiWat, group="Reference terminal unit performance", tab="Advanced"));
    parameter Modelica.Units.SI.MassFlowRate mLoaCoo_flow_nominal_ref(min=Modelica.Constants.eps) = 0.5
      "Load side mass flow rate at nominal conditions in cooling mode of reference terminal unit"
      annotation(Dialog(enable=have_chiWat, group="Reference terminal unit performance", tab="Advanced"));
    parameter Modelica.Units.SI.HeatFlowRate QCoo_flow_nominal_ref(max=-Modelica.Constants.eps) = -5.8E3
      "Heat flow at nominal conditions in cooling mode of reference terminal unit"
      annotation(Dialog(enable=have_chiWat, group="Reference terminal unit performance", tab="Advanced"));

    parameter Modelica.Units.SI.HeatFlowRate QCoo_flow_nominal(max=0)=
      if have_chiWat then
      Buildings.DHC.Loads.BaseClasses.getPeakLoad(string=
      "#Peak space cooling load",
      filNam=Modelica.Utilities.Files.loadResource(filNam))
      else 0
      "Design cooling heat flow rate (<=0)"
      annotation (Dialog(group="Nominal condition", enable=have_chiWat));
    parameter Modelica.Units.SI.HeatFlowRate QHea_flow_nominal(min=0)=
      if have_heaWat then
      Buildings.DHC.Loads.BaseClasses.getPeakLoad(string=
      "#Peak space heating load",
      filNam=Modelica.Utilities.Files.loadResource(filNam))
      else 0
      "Design heating heat flow rate (>=0)"
      annotation (Dialog(group="Nominal condition"));
    parameter Modelica.Units.SI.MassFlowRate mChiWat_flow_nominal(min=0)=
        QCoo_flow_nominal/cp_default/(T_aChiWat_nominal - T_bChiWat_nominal)
      "Chilled water mass flow rate at nominal conditions (all units)"
      annotation (Dialog(group="Nominal condition"));
    parameter Modelica.Units.SI.MassFlowRate mHeaWat_flow_nominal(min=0)=
        QHea_flow_nominal/cp_default/(T_aHeaWat_nominal - T_bHeaWat_nominal)
      "Heating water mass flow rate at nominal conditions (all units)"
      annotation (Dialog(group="Nominal condition"));
    parameter Real k(
      min=0)=0.1
      "Gain of controller";
    parameter Modelica.Units.SI.Time Ti(min=Modelica.Constants.small) = 10
      "Time constant of integrator block";

    Buildings.Controls.OBC.CDL.Interfaces.RealOutput QReqHotWat_flow(
      final unit="W") if have_hotWat
      "SHW load" annotation (Placement(
        transformation(extent={{300,-140},{340,-100}}), iconTransformation(
        extent={{-40,-40},{40,40}},
        rotation=-90,
        origin={280,-340})));
    Buildings.Controls.OBC.CDL.Interfaces.RealOutput QReqHea_flow(
      final quantity="HeatFlowRate",
      final unit="W") if have_heaLoa
      "Heating load"
      annotation (Placement(transformation(extent={{300,20},{340,60}}),
        iconTransformation(extent={{-40,-40},{40,40}},rotation=-90,origin={200,-340})));
    Buildings.Controls.OBC.CDL.Interfaces.RealOutput QReqCoo_flow(
      final quantity="HeatFlowRate",
      final unit="W") if have_cooLoa
      "Cooling load"
      annotation (Placement(transformation(extent={{300,-20},{340,20}}),
        iconTransformation(extent={{-40,-40},{40,40}},rotation=-90,origin={240,-340})));
    Modelica.Blocks.Sources.CombiTimeTable loa(
      tableOnFile=true,
      tableName="tab1",
      fileName=Modelica.Utilities.Files.loadResource(
        filNam),
      extrapolation=Modelica.Blocks.Types.Extrapolation.Periodic,
      y(each unit="W"),
      offset={0,0,0},
      columns={2,3,4},
      smoothness=Modelica.Blocks.Types.Smoothness.MonotoneContinuousDerivative1)
      "Reader for thermal loads (y[1] is cooling load, y[2] is space heating load, y[3] is domestic water heat load)"
      annotation (Placement(transformation(extent={{-280,-10},{-260,10}})));
    Buildings.Controls.OBC.CDL.Reals.Sources.Constant minTSet(
      k=293.15,
      y(final unit="K",
        displayUnit="degC"))
      if have_heaWat
      "Minimum temperature set point"
      annotation (Placement(transformation(extent={{-280,170},{-260,190}})));
    Buildings.Controls.OBC.CDL.Reals.Sources.Constant maxTSet(
      k=297.15,
      y(final unit="K",
        displayUnit="degC"))
      if have_chiWat
      "Maximum temperature set point"
      annotation (Placement(transformation(extent={{-280,210},{-260,230}})));
    replaceable Buildings.DHC.Loads.BaseClasses.Validation.BaseClasses.FanCoil2PipeHeating terUniHea(
      final k=k,
      final Ti=Ti) if have_heaWat
    constrainedby Buildings.DHC.Loads.BaseClasses.PartialTerminalUnit(
      redeclare final package Medium1=Medium,
      redeclare final package Medium2=Medium2,
      final allowFlowReversal=allowFlowReversal,
      final facMul=facMulHea,
      final facMulZon=1,
      final QHea_flow_nominal=QHea_flow_nominal/facMulHea,
      final mLoaHea_flow_nominal=mLoaHea_flow_nominal,
      final T_aHeaWat_nominal=T_aHeaWat_nominal,
      final T_bHeaWat_nominal=T_bHeaWat_nominal,
      final T_aLoaHea_nominal=T_aLoaHea_nominal)
      "Heating terminal unit"
      annotation (Placement(transformation(extent={{70,-22},{90,-2}})));
    Buildings.DHC.Loads.BaseClasses.FlowDistribution disFloHea(
      redeclare final package Medium=Medium,
      final allowFlowReversal=allowFlowReversal,
      m_flow_nominal=mHeaWat_flow_nominal,
      have_pum=true,
      typCtr=Buildings.DHC.Loads.BaseClasses.Types.PumpControlType.ConstantHead,
      dp_nominal=100000,
      nPorts_a1=1,
      nPorts_b1=1) if have_heaWat
      "Heating water distribution system"
      annotation (Placement(transformation(extent={{120,-70},{140,-50}})));
    Buildings.DHC.Loads.BaseClasses.FlowDistribution disFloCoo(
      redeclare final package Medium=Medium,
      final allowFlowReversal=allowFlowReversal,
      m_flow_nominal=mChiWat_flow_nominal,
      typDis=Buildings.DHC.Loads.BaseClasses.Types.DistributionType.ChilledWater,
      have_pum=true,
      typCtr=Buildings.DHC.Loads.BaseClasses.Types.PumpControlType.ConstantHead,
      dp_nominal=100000,
      nPorts_b1=1,
      nPorts_a1=1) if have_chiWat
      "Chilled water distribution system"
      annotation (Placement(transformation(extent={{120,-270},{140,-250}})));
    replaceable
      Buildings.DHC.Loads.BaseClasses.Validation.BaseClasses.FanCoil2PipeCooling
      terUniCoo(
      final k=k,
      final Ti=Ti,
      final QEnv_flow_nominal=if have_heaWat then QHea_flow_nominal/facMulHea else -QCoo_flow_nominal/facMulCoo)
        if have_chiWat
      constrainedby Buildings.DHC.Loads.BaseClasses.PartialTerminalUnit(
      redeclare final package Medium1 = Medium,
      redeclare final package Medium2 = Medium2,
      final allowFlowReversal=allowFlowReversal,
      final facMul=facMulCoo,
      final facMulZon=1,
      final QCoo_flow_nominal=QCoo_flow_nominal/facMulCoo,
      final mLoaCoo_flow_nominal=mLoaCoo_flow_nominal,
      final T_aChiWat_nominal=T_aChiWat_nominal,
      final T_bChiWat_nominal=T_bChiWat_nominal,
      final T_aLoaCoo_nominal=T_aLoaCoo_nominal,
      final w_aLoaCoo_nominal=w_aLoaCoo_nominal) "Cooling terminal unit"
      annotation (Placement(transformation(extent={{70,36},{90,56}})));
    Buildings.Controls.OBC.CDL.Reals.Add addPPum
      "Sum pump power"
      annotation (Placement(transformation(extent={{240,70},{260,90}})));
    Buildings.Controls.OBC.CDL.Reals.Sources.Constant noCoo(
      k=0) if not have_chiWat
      "No cooling system"
      annotation (Placement(transformation(extent={{130,70},{150,90}})));
    Buildings.Controls.OBC.CDL.Reals.Sources.Constant noHea(
      k=0) if not have_heaWat
      "No heating system"
      annotation (Placement(transformation(extent={{130,110},{150,130}})));
    Buildings.Controls.OBC.CDL.Reals.Add addPFan
      "Sum fan power"
      annotation (Placement(transformation(extent={{240,110},{260,130}})));
    Buildings.Controls.OBC.CDL.Reals.MultiplyByParameter mulQReqHea_flow(
      u(final unit="W"),
      final k=facMul) if have_heaLoa "Scaling"
      annotation (Placement(transformation(extent={{272,30},{292,50}})));
    Buildings.Controls.OBC.CDL.Reals.MultiplyByParameter mulQReqCoo_flow(u(
          final unit="W"), final k=facMul) if have_cooLoa "Scaling"
      annotation (Placement(transformation(extent={{272,-10},{292,10}})));
    Buildings.Controls.OBC.CDL.Reals.MultiplyByParameter mulQReqHot_flow(u(final
          unit="W"), final k=facMul) if have_heaLoa "Scaling"
      annotation (Placement(transformation(extent={{270,-130},{290,-110}})));
  protected
    parameter Modelica.Units.SI.AbsolutePressure pSat_nominal=
      Buildings.Utilities.Psychrometrics.Functions.saturationPressure(T_aChiWat_nominal)
      "Saturation pressure at entering water temperature";
    parameter Modelica.Units.SI.AbsolutePressure pSat_nominal_ref=
      Buildings.Utilities.Psychrometrics.Functions.saturationPressure(T_aChiWat_nominal_ref)
      "Saturation pressure at entering water temperature for reference terminal unit";
    parameter Modelica.Units.SI.MassFraction X1_aLoaCoo_nominal=
       w_aLoaCoo_nominal / (1 + w_aLoaCoo_nominal)
       "Water vapor concentration in [kg/kg total air]";
    parameter Modelica.Units.SI.MassFraction X1Sat_nominal=
      Buildings.Utilities.Psychrometrics.Functions.X_pSatpphi(
        pSat=pSat_nominal, p=Medium2.p_default, phi=1.0)
      "Water vapor concentration at saturation in [kg/kg total air]";
    parameter Modelica.Units.SI.MassFraction X1_aLoaCoo_nominal_ref=
       w_aLoaCoo_nominal_ref / (1 + w_aLoaCoo_nominal_ref)
       "Water vapor concentration in [kg/kg total air]";
    parameter Modelica.Units.SI.MassFraction X1Sat_nominal_ref=
      Buildings.Utilities.Psychrometrics.Functions.X_pSatpphi(
        pSat=pSat_nominal_ref, p=Medium2.p_default, phi=1.0)
      "Water vapor concentration at saturation in [kg/kg total air]";
    parameter Modelica.Units.SI.SpecificEnthalpy h_aLoaCoo_nominal=
      Buildings.Media.Air.specificEnthalpy_pTX(
        p=Medium2.p_default, T=T_aLoaCoo_nominal, X={X1_aLoaCoo_nominal, 1-X1_aLoaCoo_nominal})
      "Specific enthalpy of enytering air at nominal conditions in cooling mode";
    parameter Modelica.Units.SI.SpecificEnthalpy hSat_nominal=
      Buildings.Media.Air.specificEnthalpy_pTX(
        p=Medium2.p_default, T=T_aChiWat_nominal, X={X1Sat_nominal, 1-X1Sat_nominal})
      "Specific enthalpy of saturated air at entering water temperature in cooling mode";
    parameter Modelica.Units.SI.SpecificEnthalpy h_aLoaCoo_nominal_ref=
      Buildings.Media.Air.specificEnthalpy_pTX(
        p=Medium2.p_default, T=T_aLoaCoo_nominal_ref, X={X1_aLoaCoo_nominal_ref, 1-X1_aLoaCoo_nominal_ref})
      "Specific enthalpy of enytering air at nominal conditions for reference terminal unit";
    parameter Modelica.Units.SI.SpecificEnthalpy hSat_nominal_ref=
      Buildings.Media.Air.specificEnthalpy_pTX(
        p=Medium2.p_default, T=T_aChiWat_nominal_ref, X={X1Sat_nominal_ref, 1-X1Sat_nominal_ref})
      "Specific enthalpy of saturated air at entering water temperature for reference terminal unit";
  initial equation
    if have_chiWat then
      assert(QCoo_flow_nominal < -Modelica.Constants.eps, "QCoo_flow_nominal must be negative.");
      assert(T_aChiWat_nominal - T_bChiWat_nominal < 0, "Temperature difference (T_aChiWat_nominal - T_bChiWat_nominal) has wrong sign.");
    end if;
    if have_heaWat then
      assert(T_aHeaWat_nominal - T_bHeaWat_nominal > 0, "Temperature difference (T_aHeaWat_nominal - T_bHeaWat_nominal) has wrong sign.");
    end if;

  equation
    connect(terUniHea.port_bHeaWat,disFloHea.ports_a1[1])
      annotation (Line(points={{90,-20.3333},{90,-20},{146,-20},{146,-54},{140,
            -54}},                                                                   color={0,127,255}));
    connect(disFloHea.ports_b1[1],terUniHea.port_aHeaWat)
      annotation (Line(points={{120,-54},{64,-54},{64,-20.3333},{70,-20.3333}},color={0,127,255}));
    connect(terUniHea.mReqHeaWat_flow,disFloHea.mReq_flow[1])
      annotation (Line(points={{90.8333,-15.3333},{100,-15.3333},{100,-64},{119,
            -64}},                                                                     color={0,0,127}));
    connect(loa.y[1],terUniCoo.QReqCoo_flow)
      annotation (Line(points={{-259,0},{40,0},{40,42.5},{69.1667,42.5}}, color={0,0,127}));
    connect(loa.y[2],terUniHea.QReqHea_flow)
      annotation (Line(points={{-259,0},{40,0},{40,-13.6667},{69.1667,-13.6667}}, color={0,0,127}));
    connect(disFloCoo.ports_b1[1],terUniCoo.port_aChiWat)
      annotation (Line(points={{120,-254},{60,-254},{60,39.3333},{70,39.3333}},color={0,127,255}));
    connect(terUniCoo.port_bChiWat,disFloCoo.ports_a1[1])
      annotation (Line(points={{90,39.3333},{160,39.3333},{160,-254},{140,-254}}, color={0,127,255}));
    connect(terUniCoo.mReqChiWat_flow,disFloCoo.mReq_flow[1])
      annotation (Line(points={{90.8333,41},{108,41},{108,-264},{119,-264}},color={0,0,127}));
    connect(minTSet.y,terUniHea.TSetHea)
      annotation (Line(points={{-258,180},{-20,180},{-20,-7},{69.1667,-7}}, color={0,0,127}));
    connect(maxTSet.y,terUniCoo.TSetCoo)
      annotation (Line(points={{-258,220},{0,220},{0,49.3333},{69.1667,49.3333}},color={0,0,127}));
    connect(disFloHea.PPum,addPPum.u1)
      annotation (Line(points={{141,-68},{170,-68},{170,86},{238,86}},color={0,0,127}));
    connect(disFloCoo.PPum,addPPum.u2)
      annotation (Line(points={{141,-268},{200,-268},{200,74},{238,74}},color={0,0,127}));
    connect(noHea.y,addPPum.u1)
      annotation (Line(points={{152,120},{170,120},{170,86},{238,86}}, color={0,0,127}));
    connect(noCoo.y,addPPum.u2)
      annotation (Line(points={{152,80},{200,80},{200,74},{238,74}}, color={0,0,127}));
    connect(noHea.y,addPFan.u1)
      annotation (Line(points={{152,120},{180,120},{180,126},{238,126}},
                                                                       color={0,0,127}));
    connect(noCoo.y,addPFan.u2)
      annotation (Line(points={{152,80},{200,80},{200,114},{238,114}},
                                                                     color={0,0,127}));
    connect(terUniCoo.PFan,addPFan.u2)
      annotation (Line(points={{90.8333,46},{160,46},{160,114},{238,114}},color={0,0,127}));
    connect(terUniHea.PFan,addPFan.u1)
      annotation (Line(points={{90.8333,-12},{180,-12},{180,126},{238,126}},color={0,0,127}));
    connect(disFloCoo.port_b, mulChiWatOut[1].port_a)
      annotation (Line(points={{140,-260},{260,-260}}, color={0,127,255}));
    connect(disFloHea.port_b, mulHeaWatOut[1].port_a)
      annotation (Line(points={{140,-60},{260,-60}}, color={0,127,255}));
    connect(mulHeaWatInl[1].port_b, disFloHea.port_a)
      annotation (Line(points={{-260,-60},{120,-60}}, color={0,127,255}));
    connect(mulChiWatInl[1].port_b, disFloCoo.port_a)
      annotation (Line(points={{-260,-260},{120,-260}}, color={0,127,255}));
    connect(addPFan.y, mulPFan.u)
      annotation (Line(points={{262,120},{268,120}}, color={0,0,127}));
    connect(addPPum.y, mulPPum.u)
      annotation (Line(points={{262,80},{268,80}}, color={0,0,127}));
    connect(mulQReqCoo_flow.y, QReqCoo_flow)
      annotation (Line(points={{294,0},{320,0}}, color={0,0,127}));
    connect(mulQReqHea_flow.y, QReqHea_flow)
      annotation (Line(points={{294,40},{320,40}}, color={0,0,127}));
    connect(loa.y[1], mulQReqCoo_flow.u)
      annotation (Line(points={{-259,0},{270,0}}, color={0,0,127}));
    connect(loa.y[2], mulQReqHea_flow.u) annotation (Line(points={{-259,0},{260,0},
            {260,40},{270,40}}, color={0,0,127}));
    connect(disFloHea.QActTot_flow, mulQHea_flow.u) annotation (Line(points={{141,
            -66},{220,-66},{220,280},{268,280}}, color={0,0,127}));
    connect(disFloCoo.QActTot_flow, mulQCoo_flow.u) annotation (Line(points={{141,
            -266},{224,-266},{224,240},{268,240}}, color={0,0,127}));
    connect(mulQReqHot_flow.y, QReqHotWat_flow)
      annotation (Line(points={{292,-120},{320,-120}}, color={0,0,127}));
    connect(mulQReqHot_flow.u, loa.y[3]) annotation (Line(points={{268,-120},{40,-120},
            {40,0},{-259,0}}, color={0,0,127}));
  annotation (
      Documentation(
        info="<html>
<p>
This is a simplified building model where the space heating and cooling
loads are provided as time series. In order to approximate the emission
characteristic of the building HVAC system,
this model uses idealized fan coil models that are parameterized with
the peak load, determined from the provided time series, and design
values of the hot water and chilled water supply and return temperatures.
</p>
<p>
The time series that provide the loads are read from the file <code>filNam</code>.
This file must have columns as shown in this example:
<pre>
#1
#Heating, cooling and domestic hot water loads
#
#First column: Seconds in the year (loads are hourly)
#Second column: cooling loads in Watts (as negative numbers).
#Third column: space heating loads in Watts
#Fourth column: domestic hot water loads in Watts
#
#Peak space cooling load = -146960 Watts
#Peak space heating load = 167690 Watts
#Peak water heating load = 9390 Watts
double tab1(8760,4)
0;0;18230;0
3600;0;17520;0
7200;0;20170;0
10800;0;22450;0
[further rows omitted]
</pre>
Specificallly, the format must be as follows:
<ul>
<li>
The first column must be the time of the year in seconds.
</li>
<li>
If <code>have_chiWat = true</code>, then the next column must be the space cooling load in Watts.
Note that cooling is a negative number.<br/>
If <code>have_chiWat = false</code>, this column must be present but it will be ignored, and hence
it can be set to any number such as <code>0</code>.
</li>
<li>
If <code>have_heaWat = true</code>, the next column must be the space heating load in Watts.<br/>
If <code>have_heaWat = false</code>, this column must be present but it will be ignored, and hence
it can be set to any number such as <code>0</code>.
</li>
<li>
If <code>have_hotWat = true</code>, the next column must be the domestic hot water load in Watts.<br/>
If <code>have_hotWat = false</code>, this column must be present but it will be ignored, and hence
it can be set to any number such as <code>0</code>.
</li>
</ul>
<p>
The entry <code>double tab1(8760,4)</code> shows how many columns and rows are present.
</p>
<p>
The header also needs to contain the lines that start with <code>#Peak</code> as shown in the example above.
</p>
<h4>Implementation details</h4>
<p>
The total space heating (resp. cooling) load is split between
<code>facMulHea</code> (resp. <code>facMulCoo</code>)
identical terminal units with heat transfer performance approximated based on
design specifications of a reference terminal unit.
It is not expected that the user modifies the default values
that are proposed for <code>facMulHea</code> and <code>facMulCoo</code>
unless detailed design data are available for the building
HVAC system.
In that latter case, the following set of parameters should be
modified consistently to match the design data.
</p>
<ul>
<li>Hot water (resp. chilled water) supply and return temperature
<code>T_aHeaWat_nominal</code> and <code>T_bHeaWat_nominal</code>
(resp. <code>T_aChiWat_nominal</code> and <code>T_bChiWat_nominal</code>)
</li>
<li>Terminal unit entering air temperature <code>T_aLoaHea_nominal</code>
(resp. <code>T_aLoaCoo_nominal</code>) and humidity ratio
<code>w_aLoaCoo_nominal</code>
</li>
<li>Terminal unit air mass flow rate <code>mLoaHea_flow_nominal</code>
(resp. <code>mLoaCoo_flow_nominal</code>)
</li>
<li>Terminal unit scaling factor <code>facMulHea</code>
(resp. <code>facMulCoo</code>)
</li>
</ul>
<p>
For reference, the default reference terminal unit performance is based on
manufacturer data (Carrier fan coil model 42NL/NH) at selection conditions
as specified in the \"Advanced\" tab.
</p>
</html>",
  revisions="<html>
<ul>
<li>
May 3, 2023, by David Blum:<br/>
Applied <code>facMul</code> to domestic hot water load.<br/>
This is for
<a href=\"https://github.com/lbl-srg/modelica-buildings/issues/3379\">
issue 3379</a>.
</li>
<li>
November 21, 2022, by David Blum:<br/>
Scale <code>facMulHea</code> and <code>facMulCoo</code> with peak load.<br/>
This is for
<a href=\"https://github.com/lbl-srg/modelica-buildings/issues/2302\">
issue 2302</a>.
</li>
<li>
December 21, 2020, by Antoine Gautier:<br/>
Refactored for optional hot water and multiplier factor.<br/>
This is for <a href=\"https://github.com/lbl-srg/modelica-buildings/issues/2291\">issue 2291</a>.
</li>
<li>
September 18, 2020, by Jianjun Hu:<br/>
Changed flow distribution components and the terminal units to be conditional depending
on if there is water-based heating, or cooling system.
This is for <a href=\"https://github.com/lbl-srg/modelica-buildings/issues/2147\">issue 2147</a>.
</li>
<li>
February 21, 2020, by Antoine Gautier:<br/>
First implementation.
</li>
</ul>
</html>"),
      Icon(
        coordinateSystem(
          preserveAspectRatio=false,
          extent={{-300,-300},{300,300}})));
  end BuildingTimeSeries;

  model SingleZoneFloor "Model of a building floor as a single zone"
    replaceable package Medium = Modelica.Media.Interfaces.PartialMedium
      "Medium model for air" annotation (choicesAllMatching=true);
    parameter Modelica.Units.SI.Volume VRoo=4555.7 "Room volum";
    parameter Modelica.Units.SI.Height hRoo=2.74 "Room height";
    parameter Modelica.Units.SI.Length hWin=1.5 "Height of windows";
    parameter Real winWalRat(min=0.01,max=0.99) = 0.33
      "Window to wall ratio for exterior walls";

    parameter Buildings.HeatTransfer.Data.Solids.Plywood matWoo(
      x=0.01,
      k=0.11,
      d=544,
      nStaRef=1) "Wood for exterior construction"
      annotation (Placement(transformation(extent={{40,110},{60,130}})));
    parameter Buildings.HeatTransfer.Data.Solids.Concrete matCon(
      x=0.1,
      k=1.311,
      c=836,
      nStaRef=5) "Concrete"
      annotation (Placement(transformation(extent={{40,140},{60,160}})));
    parameter Buildings.HeatTransfer.Data.Solids.Generic matIns(
      x=0.087,
      k=0.049,
      c=836.8,
      d=265,
      nStaRef=5) "Steelframe construction with insulation"
      annotation (Placement(transformation(extent={{80,110},{100,130}})));
    parameter Buildings.HeatTransfer.Data.Solids.GypsumBoard matGyp(
      x=0.0127,
      k=0.16,
      c=830,
      d=784,
      nStaRef=2) "Gypsum board"
      annotation (Placement(transformation(extent={{40,80},{60,100}})));
    parameter Buildings.HeatTransfer.Data.Solids.GypsumBoard matGyp2(
      x=0.025,
      k=0.16,
      c=830,
      d=784,
      nStaRef=2) "Gypsum board"
      annotation (Placement(transformation(extent={{80,80},{100,100}})));
    parameter Buildings.HeatTransfer.Data.Solids.Plywood matFur(x=0.15)
      "Material for furniture"
      annotation (Placement(transformation(extent={{80,170},{100,190}})));
    parameter Buildings.HeatTransfer.Data.Resistances.Carpet matCar "Carpet"
      annotation (Placement(transformation(extent={{120,140},{140,160}})));
    parameter Buildings.HeatTransfer.Data.GlazingSystems.DoubleClearAir13Clear glaSys(
      UFra=2,
      shade=Buildings.HeatTransfer.Data.Shades.Gray(),
      haveInteriorShade=false,
      haveExteriorShade=false) "Data record for the glazing system"
      annotation (Placement(transformation(extent={{80,140},{100,160}})));
    parameter Buildings.HeatTransfer.Data.OpaqueConstructions.Generic conExtWal(
      final nLay=3,
      material={matWoo,matIns,matGyp}) "Exterior construction"
      annotation (Placement(transformation(extent={{120,110},{140,130}})));
    parameter Buildings.HeatTransfer.Data.OpaqueConstructions.Generic conIntWal(
      final nLay=1,
      material={matGyp2}) "Interior wall construction"
      annotation (Placement(transformation(extent={{160,110},{180,130}})));
    parameter Buildings.HeatTransfer.Data.OpaqueConstructions.Generic conFlo(
      final nLay=1,
      material={matCon}) "Floor construction (opa_a is carpet)"
      annotation (Placement(transformation(extent={{120,80},{140,100}})));
    parameter Buildings.HeatTransfer.Data.OpaqueConstructions.Generic conFur(
      final nLay=1,
      material={matFur}) "Construction for internal mass of furniture"
      annotation (Placement(transformation(extent={{160,80},{180,100}})));

    parameter Boolean use_windPressure=true
      "Set to true to enable wind pressure";

    Modelica.Fluid.Vessels.BaseClasses.VesselFluidPorts_b ports[2](
      redeclare package Medium = Medium) "Fluid inlets and outlets" annotation (Placement(
          transformation(extent={{-210,-8},{-170,8}}), iconTransformation(extent={
              {-132,-128},{-92,-112}})));

    Modelica.Blocks.Interfaces.RealOutput TRooAir(
      unit="K",
      displayUnit="degC") "Room air temperature"
      annotation (Placement(transformation(extent={{200,-10},{220,10}}),
          iconTransformation(extent={{160,92},{180,112}})));
    Modelica.Blocks.Interfaces.RealOutput p_rel
      "Relative pressure signal of building static pressure" annotation (
        Placement(transformation(extent={{200,-70},{220,-50}}),
          iconTransformation(extent={{160,-110},{180,-90}})));

    Buildings.ThermalZones.Detailed.MixedAir flo(
      redeclare package Medium = Medium,
      AFlo=AFlo,
      hRoo=hRoo,
      nConExt=0,
      nConExtWin=4,
      datConExtWin(
        layers={conExtWal,conExtWal,conExtWal,conExtWal},
        A={49.91*hRoo,49.91*hRoo,33.27*hRoo,33.27*hRoo},
        glaSys={glaSys,glaSys,glaSys,glaSys},
        wWin={winWalRat/hWin*49.91*hRoo,winWalRat/hWin*49.91*hRoo,winWalRat/hWin*33.27
            *hRoo,winWalRat/hWin*33.27*hRoo},
        each hWin=hWin,
        fFra={0.1,0.1,0.1,0.1},
        til={Z_,Z_,Z_,Z_},
        azi={N_,S_,W_,E_}),
      nConPar=3,
      datConPar(
        layers={conFlo,conFur,conIntWal},
        A={AFlo,AFlo*2,(6.47*2 + 40.76 + 24.13)*2*hRoo},
        til={F_,Z_,Z_}),
      nConBou=0,
      nSurBou=0,
      nPorts=7,
      intConMod=Buildings.HeatTransfer.Types.InteriorConvection.Temperature,
      energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial) "Floor"
      annotation (Placement(transformation(extent={{-16,-56},{24,-16}})));
    Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor temAir
      "Air temperature sensor"
      annotation (Placement(transformation(extent={{80,-10},{100,10}})));
    Buildings.Fluid.Sensors.RelativePressure senRelPre(
      redeclare package Medium = Medium)
      "Building pressure measurement"
      annotation (Placement(transformation(extent={{-60,-16},{-80,4}})));
    Buildings.Fluid.Sources.Outside out(
      nPorts=1, redeclare package Medium = Medium) "Outdoor air"
      annotation (Placement(transformation(extent={{-120,-16},{-100,4}})));
    Buildings.BoundaryConditions.WeatherData.Bus weaBus "Weather data bus"
      annotation (Placement(transformation(extent={{-174,78},{-158,94}}),
          iconTransformation(extent={{-140,162},{-124,178}})));

    Buildings.Examples.VAVReheat.BaseClasses.RoomLeakage leaSou(
      redeclare package Medium = Medium,
      VRoo=568.77,
      s=49.91/33.27,
      azi=S_,
      final use_windPressure=use_windPressure)
      "Model for air infiltration through the envelope"
      annotation (Placement(transformation(extent={{-122,132},{-86,172}})));
    Buildings.Examples.VAVReheat.BaseClasses.RoomLeakage leaEas(
      redeclare package Medium = Medium,
      VRoo=360.0785,
      s=33.27/49.91,
      azi=E_,
      final use_windPressure=use_windPressure)
      "Model for air infiltration through the envelope"
      annotation (Placement(transformation(extent={{-122,92},{-86,132}})));
    Buildings.Examples.VAVReheat.BaseClasses.RoomLeakage leaNor(
      redeclare package Medium = Medium,
      VRoo=568.77,
      s=49.91/33.27,
      azi=N_,
      final use_windPressure=use_windPressure)
      "Model for air infiltration through the envelope"
      annotation (Placement(transformation(extent={{-122,50},{-86,90}})));
    Buildings.Examples.VAVReheat.BaseClasses.RoomLeakage leaWes(
      redeclare package Medium = Medium,
      VRoo=360.0785,
      s=33.27/49.91,
      azi=W_,
      final use_windPressure=use_windPressure)
      "Model for air infiltration through the envelope"
      annotation (Placement(transformation(extent={{-122,10},{-86,50}})));
    Modelica.Blocks.Sources.CombiTimeTable intGaiFra(
      table=[0,0.05; 8,0.05; 9,0.9; 12,0.9; 12,0.8; 13,0.8; 13,1; 17,1; 19,0.1;
          24,0.05],
      timeScale=3600,
      extrapolation=Modelica.Blocks.Types.Extrapolation.Periodic)
      "Fraction of internal heat gain"
      annotation (Placement(transformation(extent={{-160,-150},{-140,-130}})));
    Modelica.Blocks.Math.MatrixGain gai(K=10*[0.4; 0.4; 0.2])
      "Matrix gain to split up heat gain in radiant, convective and latent gain"
      annotation (Placement(transformation(extent={{-120,-150},{-100,-130}})));
    Modelica.Blocks.Sources.Constant uSha(k=0)
      "Control signal for the shading device"
      annotation (Placement(transformation(extent={{-160,-100},{-140,-80}})));
    Modelica.Blocks.Routing.Replicator replicator(nout=1)
      "Shading signals for all windows"
      annotation (Placement(transformation(extent={{-120,-100},{-100,-80}})));

  protected
    parameter Modelica.Units.SI.Angle S_=Buildings.Types.Azimuth.S
      "Azimuth for south walls";
    parameter Modelica.Units.SI.Angle E_=Buildings.Types.Azimuth.E
      "Azimuth for east walls";
    parameter Modelica.Units.SI.Angle W_=Buildings.Types.Azimuth.W
      "Azimuth for west walls";
    parameter Modelica.Units.SI.Angle N_=Buildings.Types.Azimuth.N
      "Azimuth for north walls";
    parameter Modelica.Units.SI.Angle F_=Buildings.Types.Tilt.Floor
      "Tilt for floor";
    parameter Modelica.Units.SI.Angle Z_=Buildings.Types.Tilt.Wall
      "Tilt for wall";
    parameter Modelica.Units.SI.Area AFlo=VRoo/hRoo "Floor area";

  equation
    connect(flo.weaBus, weaBus) annotation (Line(
        points={{21.9,-18.1},{21.9,184},{-166,184},{-166,86}},
        color={255,204,51},
        thickness=0.5,
        smooth=Smooth.None));
    connect(flo.heaPorAir, temAir.port) annotation (Line(
        points={{3,-36},{40,-36},{40,0},{80,0}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(out.weaBus, weaBus) annotation (Line(
        points={{-120,-5.8},{-166,-5.8},{-166,86}},
        color={255,204,51},
        thickness=0.5,
        smooth=Smooth.None), Text(
        textString="%second",
        index=1,
        extent={{6,3},{6,3}}));
    connect(out.ports[1], senRelPre.port_b) annotation (Line(
        points={{-100,-6},{-80,-6}},
        color={0,127,255},
        smooth=Smooth.None,
        thickness=0.5));
    connect(temAir.T, TRooAir) annotation (Line(points={{101,0},{210,0}},
                      color={0,0,127}));
    connect(weaBus, leaSou.weaBus) annotation (Line(
        points={{-166,86},{-166,152},{-122,152}},
        color={255,204,51},
        thickness=0.5), Text(
        string="%first",
        index=-1,
        extent={{-6,3},{-6,3}},
        horizontalAlignment=TextAlignment.Right));
    connect(weaBus, leaEas.weaBus) annotation (Line(
        points={{-166,86},{-166,112},{-122,112}},
        color={255,204,51},
        thickness=0.5), Text(
        string="%first",
        index=-1,
        extent={{-6,3},{-6,3}},
        horizontalAlignment=TextAlignment.Right));
    connect(leaNor.weaBus, weaBus) annotation (Line(
        points={{-122,70},{-166,70},{-166,86}},
        color={255,204,51},
        thickness=0.5), Text(
        string="%second",
        index=1,
        extent={{-6,3},{-6,3}},
        horizontalAlignment=TextAlignment.Right));
    connect(leaSou.port_b, flo.ports[1]) annotation (Line(points={{-86,152},{
            -44,152},{-44,-47.7143},{-11,-47.7143}},
                                            color={0,127,255}));
    connect(leaEas.port_b, flo.ports[2]) annotation (Line(points={{-86,112},{
            -44,112},{-44,-37.3333},{-11,-37.3333},{-11,-47.1429}},
                                                      color={0,127,255}));
    connect(leaNor.port_b, flo.ports[3]) annotation (Line(points={{-86,70},{-44,
            70},{-44,-40},{-11,-40},{-11,-46.5714}},
                                                 color={0,127,255}));
    connect(leaWes.port_b, flo.ports[4]) annotation (Line(points={{-86,30},{-44,
            30},{-44,-46},{-11,-46}},       color={0,127,255}));
    connect(senRelPre.port_a, flo.ports[5]) annotation (Line(points={{-60,-6},{
            -44,-6},{-44,-44},{-11,-44},{-11,-45.4286}},       color={0,127,255}));
    connect(weaBus, leaWes.weaBus) annotation (Line(
        points={{-166,86},{-166,30},{-122,30}},
        color={255,204,51},
        thickness=0.5), Text(
        string="%first",
        index=-1,
        extent={{-6,3},{-6,3}},
        horizontalAlignment=TextAlignment.Right));
    connect(senRelPre.p_rel, p_rel) annotation (Line(points={{-70,-15},{-70,-60},{210,-60}}, color={0,0,127}));
    connect(intGaiFra.y, gai.u)  annotation (Line(points={{-139,-140},{-122,-140}}, color={0,0,127}));
    connect(gai.y, flo.qGai_flow) annotation (Line(points={{-99,-140},{-26,-140},
            {-26,-28},{-17.6,-28}},
                                  color={0,0,127}));
    connect(uSha.y, replicator.u)  annotation (Line(points={{-139,-90},{-122,-90}}, color={0,0,127}));
    connect(replicator.y, flo.uSha) annotation (Line(points={{-99,-90},{-30,-90},
            {-30,-18},{-17.6,-18}},
                                color={0,0,127}));
    connect(ports[1], flo.ports[6]) annotation (Line(points={{-195,0},{-188,0},
            {-188,-50},{-11,-50},{-11,-44.8571}},
                                         color={0,127,255}));
    connect(ports[2], flo.ports[7]) annotation (Line(points={{-185,0},{-192,0},
            {-192,-44.2857},{-11,-44.2857}},
                                 color={0,127,255}));
    annotation (
    Diagram(coordinateSystem(preserveAspectRatio=true, extent={{-200,-200},{200,200}})),
    defaultComponentName="sinZonFlo",
    Documentation(info="<html>
  <p>
  This model assumes a mid-floor of a building as a single zone with a homogeneous 
  temperature; i.e., the air in the whole floor is assumed to be fully mixed.
  </p>
  <p>
  The geometry, materials and constructions of the model are consistent with those of
  <a href=\"modelica://Buildings.Examples.VAVReheat.BaseClasses.Floor\">
  Buildings.Examples.VAVReheat.BaseClasses.Floor</a>, 
  which models the same mid-floor as five zones: a core zone and 
  four perimeter zones.
  </p>
  <p>
  The internal partition walls in the five-zone floor model are considered as
  thermal mass in this single-zone floor model. The doors in the five-zone floor 
  model have been removed in the single-zone floor model.
  </p>
  </html>",
    revisions="<html>
<ul>
<li>
September 16, 2021, by Michael Wetter:<br/>
Removed parameter <code>lat</code> as this is now obtained from the weather data reader.<br/>
This is for
<a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/1477\">IBPSA, #1477</a>.
</li>
<li>
March 10, 2020, by Kun Zhang:<br/>
First implementation. This is for issue
<a href=\"https://github.com/lbl-srg/modelica-buildings/issues/1717\">1717</a>.
</li>
</ul>
</html>"),
    Icon(coordinateSystem(preserveAspectRatio=true, extent={{-200,-200},{200,200}}),
          graphics={
          Rectangle(
            extent={{-160,-160},{160,160}},
            lineColor={95,95,95},
            fillColor={95,95,95},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{-140,140},{140,-140}},
            pattern=LinePattern.None,
            lineColor={117,148,176},
            fillColor={170,213,255},
            fillPattern=FillPattern.Sphere),
          Rectangle(
            extent={{140,70},{160,-70}},
            lineColor={95,95,95},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{146,70},{154,-70}},
            lineColor={95,95,95},
            fillColor={170,213,255},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{-160,72},{-140,-68}},
            lineColor={95,95,95},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{-154,72},{-146,-68}},
            lineColor={95,95,95},
            fillColor={170,213,255},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{-10,70},{10,-70}},
            lineColor={95,95,95},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid,
            origin={2,-150},
            rotation=90),
          Rectangle(
            extent={{-4,70},{4,-70}},
            lineColor={95,95,95},
            fillColor={170,213,255},
            fillPattern=FillPattern.Solid,
            origin={2,-150},
            rotation=90),
          Rectangle(
            extent={{-10,70},{10,-70}},
            lineColor={95,95,95},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid,
            origin={2,150},
            rotation=90),
          Rectangle(
            extent={{-4,70},{4,-70}},
            lineColor={95,95,95},
            fillColor={170,213,255},
            fillPattern=FillPattern.Solid,
            origin={2,150},
            rotation=90),
            Text(
              extent={{-100,238},{100,184}},
              textColor={0,0,255},
            textString="%name")}),
                Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  end SingleZoneFloor;

  model StorageTankWithExternalHeatExchanger1
    "A model of a storage tank with external heat exchanger to produce hot water"
    extends Buildings.DHC.Loads.HotWater.BaseClasses.PartialFourPortDHW(
      final allowFlowReversalHea=false);

    parameter Buildings.DHC.Loads.HotWater.Data.GenericDomesticHotWaterWithHeatExchanger
      dat "Performance data"
      annotation (Placement(transformation(extent={{60,80},{80,100}})));

    parameter Real k=0.1 "Proportional gain of circulation pump controller";
    parameter Real Ti=60 "Integrator time constant of circulation pump controller";

    parameter Modelica.Media.Interfaces.Types.Temperature TTan_start=323.15
      "Start value of tank temperature"
      annotation(Dialog(tab="Initialization"));
    final parameter Real eps =
      dat.QHex_flow_nominal / CMin_flow_nominal / ( dat.TDom_nominal + dat.dTHexApp_nominal - dat.TCol_nominal)
      "Heat exchanger effectiveness"
      annotation(Dialog(tab="Advanced"));
    Buildings.Fluid.Movers.Preconfigured.FlowControlled_dp pumHex(
      redeclare package Medium = MediumHea,
      energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
      use_inputFilter=false,
      riseTime=10,
      m_flow_nominal=dat.mHex_flow_nominal,
      dp_nominal=dat.dpHexHea_nominal) "Pump with head as input" annotation (
        Placement(transformation(
          extent={{-10,10},{10,-10}},
          rotation=270,
          origin={-40,0})));

    Modelica.Blocks.Interfaces.RealOutput PEle(unit="W")
      "Electric power required for pumping equipment"
      annotation (Placement(transformation(extent={{100,-10},{120,10}}),
          iconTransformation(extent={{100,-10},{120,10}})));
    Buildings.Fluid.Sensors.TemperatureTwoPort senTemHot(
      redeclare package Medium =  MediumDom,
      final allowFlowReversal=allowFlowReversalDom,
      m_flow_nominal=dat.mDom_flow_nominal)
      "Temperature sensor for hot water supply"
      annotation (Placement(transformation(extent={{20,50},{40,70}})));

    Buildings.Fluid.HeatExchangers.ConstantEffectiveness hex(
      redeclare package Medium1 = MediumDom,
      redeclare package Medium2 = MediumHea,
      final allowFlowReversal1=allowFlowReversalDom,
      m1_flow_nominal=dat.mDom_flow_nominal,
      m2_flow_nominal=dat.mHex_flow_nominal,
      dp1_nominal=dat.dpHexHea_nominal,
      from_dp2=true,
      dp2_nominal=dat.dpHexDom_nominal,
      eps=eps)
      annotation (Placement(transformation(extent={{-60,44},{-40,64}})));
    Buildings.Fluid.FixedResistances.Junction junTop(
      redeclare package Medium = MediumHea,
      energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
      m_flow_nominal=dat.mHex_flow_nominal*{1,1,1},
      dp_nominal=zeros(3)) "Flow junction at top of tank"
      annotation (Placement(transformation(extent={{10,20},{30,40}})));
    Buildings.Fluid.Sensors.MassFlowRate senMasFlo(redeclare package Medium = MediumDom)
      "Mass flow rate of domestic hot water"
      annotation (Placement(transformation(extent={{-20,50},{0,70}})));
    Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor TTanTop(
      T(displayUnit="degC"))
      "Fluid temperature at the top of the tank"
      annotation (Placement(transformation(extent={{40,0},{60,20}})));
    Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor TTanBot(
      T(displayUnit="degC"))
      "Fluid temperature at the bottom of the tank"
      annotation (Placement(transformation(extent={{40,-30},{60,-10}})));

    Buildings.DHC.Loads.HotWater.BaseClasses.HeatExchangerPumpController conPum(final mDom_flow_nominal=dat.mDom_flow_nominal,
        final dpPum_nominal=dat.dpHexHea_nominal)
      "Controller for pump of heat exchanger"
      annotation (Placement(transformation(extent={{-80,-10},{-60,10}})));
    Buildings.DHC.Loads.HotWater.BaseClasses.TankChargingController conCha "Controller for tank charge signal"
      annotation (Placement(transformation(extent={{72,-90},{92,-70}})));

    Buildings.Controls.OBC.CDL.Interfaces.BooleanOutput charge
      "Output true if tank needs to be charged, false if it is sufficiently charged"
      annotation (Placement(transformation(extent={{100,-100},{140,-60}}),
          iconTransformation(extent={{100,-110},{140,-70}})));
    Modelica.Blocks.Sources.RealExpression realExpression2(y=343)
      annotation (Placement(transformation(extent={{-796,-12},{-776,8}})));
    Modelica.Blocks.Sources.RealExpression realExpression1(y=333)
      annotation (Placement(transformation(extent={{32,-82},{52,-62}})));
    ProsNet.Under_Development.Storage.StratifiedHeatStorage
                                  tan(
      redeclare package Medium = Buildings.Media.Water,
      m_flow_nominal=2.5,
      VTan(displayUnit="l") = 0.155,
      hTan=3,
      dIns=0.3,
      nSeg=10,
      T_start=323.15)
      annotation (Placement(transformation(extent={{-4,-22},{24,6}})));
  protected
    parameter Modelica.Units.SI.SpecificHeatCapacity cpHea_default =
      MediumHea.specificHeatCapacityCp(MediumHea.setState_pTX(
        MediumHea.p_default,
        MediumHea.T_default,
        MediumHea.X_default))
      "Specific heat capacity of heating medium at default medium state";
    parameter Modelica.Units.SI.SpecificHeatCapacity cpDom_default =
      MediumDom.specificHeatCapacityCp(MediumDom.setState_pTX(
        MediumDom.p_default,
        MediumDom.T_default,
        MediumDom.X_default))
      "Specific heat capacity of domestic hot water medium at default medium state";
    parameter Modelica.Units.SI.ThermalConductance CMin_flow_nominal =
      min(dat.mHex_flow_nominal*cpHea_default, dat.mDom_flow_nominal*cpDom_default)
      "Minimum heat capacity flow rate";
  initial equation
    assert(eps < 1, "In " + getInstanceName() + ": Heat exchanger effectivness must be below 1, received eps = " + String(eps) + ". Check sizing.");

  equation
    connect(pumHex.P, PEle) annotation (Line(points={{-49,-11},{-50,-11},{-50,-32},
            {86,-32},{86,0},{110,0}},     color={0,0,127}));
    connect(junTop.port_2, port_aHea) annotation (Line(points={{30,30},{84,30},{84,
            -60},{100,-60}},     color={0,127,255}));
    connect(hex.port_b1, senMasFlo.port_a)
      annotation (Line(points={{-40,60},{-20,60}}, color={0,127,255}));
    connect(senMasFlo.port_b, senTemHot.port_a)
      annotation (Line(points={{0,60},{20,60}},    color={0,127,255}));
    connect(senMasFlo.m_flow, conPum.mDom_flow) annotation (Line(points={{-10,71},
            {-10,74},{-86,74},{-86,6},{-82,6}}, color={0,0,127}));
    connect(senTemHot.T, conPum.TDom) annotation (Line(points={{30,71},{30,80},{-88,
            80},{-88,-6},{-81,-6}}, color={0,0,127}));
    connect(conPum.TDomSet, TDomSet) annotation (Line(points={{-81,0},{-92,0},{-92,
            0},{-110,0}}, color={0,0,127}));
    connect(conCha.TTanTop, TTanTop.T) annotation (Line(points={{70,-80.6},{66,
            -80.6},{66,10},{61,10}},
                                color={0,0,127}));
    connect(conCha.charge, charge) annotation (Line(points={{94,-80},{120,-80}},
                             color={255,0,255}));
    connect(senTemHot.port_b, port_bDom)
      annotation (Line(points={{40,60},{100,60}}, color={0,127,255}));
    connect(conPum.dpPumHex, pumHex.dp_in)
      annotation (Line(points={{-58,0},{-52,0}}, color={0,0,127}));

    connect(port_aDom, hex.port_a1)
      annotation (Line(points={{-100,60},{-60,60}}, color={0,127,255}));
    connect(junTop.port_1, hex.port_a2) annotation (Line(points={{10,30},{-30,30},
            {-30,48},{-40,48}}, color={0,127,255}));
    connect(hex.port_b2, pumHex.port_a) annotation (Line(points={{-60,48},{-72,48},
            {-72,20},{-40,20},{-40,10}}, color={0,127,255}));
    connect(TTanBot.T, conCha.TTanBot) annotation (Line(points={{61,-20},{62,-20},
            {62,-88},{70,-88}}, color={0,0,127}));
    connect(realExpression1.y, conCha.TTanTopSet)
      annotation (Line(points={{53,-72},{62,-72},{62,-84.8},{70.8,-84.8}},
                                                   color={0,0,127}));
    connect(tan.port_a, junTop.port_3) annotation (Line(points={{10,6},{10,14},
            {20,14},{20,20}}, color={0,127,255}));
    connect(tan.port_b, port_bHea) annotation (Line(points={{10,-22},{10,-60},{
            -100,-60}}, color={0,127,255}));
    connect(tan.fluPorVol1[8], pumHex.port_b) annotation (Line(points={{15.04,
            -6.32},{26,-6.32},{26,-24},{-40,-24},{-40,-10}}, color={0,127,255}));
    connect(tan.heaPorTop, TTanTop.port) annotation (Line(points={{12.8,2.36},{
            12.8,10},{40,10}}, color={191,0,0}));
    connect(tan.heaPorBot, TTanBot.port) annotation (Line(points={{12.8,-18.36},
            {12.8,-28},{34,-28},{34,-20},{40,-20}}, color={191,0,0}));
    annotation (
    defaultComponentName="domHotWatTan",
    Documentation(info="<html>
<p>
This model implements a heating hot water tank with external heat exchanger that heats domestic hot water.
</p>
<p>
The storage tank model is described in
<a href=\"modelica://Buildings.Fluid.Storage.StratifiedEnhancedInternalHex\">
Buildings.Fluid.Storage.StratifiedEnhancedInternalHex</a>.
The heat pump and storage tank system should be parameterized altogether using
<a href=\"modelica://Buildings.DHC.Loads.HotWater.Data.GenericDomesticHotWaterWithHeatExchanger\">
Buildings.DHC.Loads.HotWater.Data.GenericDomesticHotWaterWithHeatExchanger</a>.
</p>
<p align=\"center\">
<img alt=\"image\" src=\"modelica://Buildings/Resources/Images/DHC/Loads/HotWater/StorageTankWithExternalHeatExchanger.png\"/>
</p>
<p>
It is based on Fig. 3 in <i>Evaluations of different domestic hot water
preparing methods with ultra-low-temperature district heating</i> by X. Yang,
H. Li, and S. Svendsen at <a href=\"https:/doi.org/10.1016/j.energy.2016.04.109\">
doi.org/10.1016/j.energy.2016.04.109</a>, as well as the
<i>Advanced Energy Design Guide for Multifamily Buildings-Achieving Zero Energy</i>
published by ASHRAE in 2022 at <a href=\"https://www.ashrae.org/technical-resources/aedgs/zero-energy-aedg-free-download\">
https://www.ashrae.org/technical-resources/aedgs/zero-energy-aedg-free-download</a>.
</p>
<p>
For a model that connects this hot water system to a heat pump, see
<a href=\"modelica://Buildings.DHC.ETS.Combined.Subsystems.HeatPumpDHWTank\">
Buildings.DHC.ETS.Combined.Subsystems.HeatPumpDHWTank</a>.
</p>
</html>",   revisions="<html>
<ul>
<li>
October 5, 2023, by Michael Wetter:<br/>
First implementation.
</li>
</ul>
</html>"),  Icon(coordinateSystem(preserveAspectRatio=false), graphics={
          Polygon(
            points={{-140,86},{-140,86}},
            lineColor={95,95,95},
            lineThickness=1,
            fillColor={0,0,0},
            fillPattern=FillPattern.CrossDiag),
          Rectangle(
            extent={{60,40},{0,-78}},
            lineColor={0,0,0},
            lineThickness=0.5),
          Rectangle(
            extent={{-30,40},{-50,0}},
            lineColor={0,0,0},
            lineThickness=0.5),
          Rectangle(
            extent={{-100,64},{-60,58}},
            fillColor={0,140,72},
            fillPattern=FillPattern.Solid,
            pattern=LinePattern.None),
          Rectangle(
            extent={{-44,64},{100,58}},
            fillColor={102,44,145},
            fillPattern=FillPattern.Solid,
            pattern=LinePattern.None),
          Rectangle(
            extent={{-60,-10},{-44,-16}},
            fillColor={0,140,72},
            fillPattern=FillPattern.Solid,
            pattern=LinePattern.None),
          Rectangle(
            extent={{-40,2},{40,-2}},
            fillColor={0,140,72},
            fillPattern=FillPattern.Solid,
            pattern=LinePattern.None,
            origin={-60,24},
            rotation=90),
          Rectangle(
            extent={{-6,2},{6,-2}},
            fillColor={0,140,72},
            fillPattern=FillPattern.Solid,
            pattern=LinePattern.None,
            origin={-46,-6},
            rotation=90),
          Rectangle(
            extent={{76,-58},{98,-62}},
            fillColor={238,46,47},
            fillPattern=FillPattern.Solid,
            pattern=LinePattern.None),
          Rectangle(
            extent={{-58,2},{58,-2}},
            fillColor={238,46,47},
            fillPattern=FillPattern.Solid,
            pattern=LinePattern.None,
            origin={22,50},
            rotation=180),
          Rectangle(
            extent={{-56.5,2.5},{56.5,-2.5}},
            fillColor={238,46,47},
            fillPattern=FillPattern.Solid,
            pattern=LinePattern.None,
            origin={77.5,-4.5},
            rotation=270),
          Rectangle(
            extent={{-6,2},{6,-2}},
            fillColor={28,108,200},
            fillPattern=FillPattern.Solid,
            pattern=LinePattern.None,
            origin={-6,-60},
            rotation=180),
          Rectangle(
            extent={{-6,2},{6,-2}},
            fillColor={238,46,47},
            fillPattern=FillPattern.Solid,
            pattern=LinePattern.None,
            origin={32,46},
            rotation=270),
          Rectangle(
            extent={{-6,2},{6,-2}},
            fillColor={238,46,47},
            fillPattern=FillPattern.Solid,
            pattern=LinePattern.None,
            origin={-34,46},
            rotation=270),
          Rectangle(
            extent={{-15,2},{15,-2}},
            fillColor={28,108,200},
            fillPattern=FillPattern.Solid,
            pattern=LinePattern.None,
            origin={-85,-60},
            rotation=180),
          Rectangle(
            extent={{-6,2},{6,-2}},
            fillColor={28,108,200},
            fillPattern=FillPattern.Solid,
            pattern=LinePattern.None,
            origin={-26,-44},
            rotation=360),
          Rectangle(
            extent={{-23,2},{23,-2}},
            fillColor={28,108,200},
            fillPattern=FillPattern.Solid,
            pattern=LinePattern.None,
            origin={-34,-23},
            rotation=270),
          Rectangle(
            extent={{-12,2},{12,-2}},
            fillColor={102,44,145},
            fillPattern=FillPattern.Solid,
            pattern=LinePattern.None,
            origin={-44,52},
            rotation=90),
          Text(
            extent={{-116,36},{-66,0}},
            textColor={0,0,127},
            textString="TDomSet"),
          Rectangle(
            extent={{-5,2},{5,-2}},
            fillColor={28,108,200},
            fillPattern=FillPattern.Solid,
            pattern=LinePattern.None,
            origin={-10,-55},
            rotation=270),
          Rectangle(
            extent={{-6,2},{6,-2}},
            fillColor={28,108,200},
            fillPattern=FillPattern.Solid,
            pattern=LinePattern.None,
            origin={-6,-12},
            rotation=180),
          Polygon(
            points={{-13,-3},{-5,3},{-5,-9},{-13,-3}},
            lineColor={0,0,0},
            fillColor={0,0,0},
            fillPattern=FillPattern.Solid,
            origin={-7,-57},
            rotation=270),
          Polygon(
            points={{-15,-3},{-7,3},{-7,-9},{-15,-3}},
            lineColor={0,0,0},
            fillColor={0,0,0},
            fillPattern=FillPattern.Solid,
            origin={-13,-29},
            rotation=90),
          Polygon(
            points={{-15,-3},{-5,3},{-5,-9},{-15,-3}},
            lineColor={0,0,0},
            fillColor={0,0,0},
            fillPattern=FillPattern.Solid,
            origin={-25,-47},
            rotation=180),
          Rectangle(
            extent={{-12,2},{12,-2}},
            fillColor={28,108,200},
            fillPattern=FillPattern.Solid,
            pattern=LinePattern.None,
            origin={-10,-24},
            rotation=270),
          Ellipse(
            extent={{-42,-20},{-26,-34}},
            lineColor={0,0,0},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid),
          Polygon(
            points={{-15,-3},{-7,5},{-7,-11},{-15,-3}},
            lineColor={0,0,0},
            fillColor={0,0,0},
            fillPattern=FillPattern.Solid,
            origin={-37,-19},
            rotation=90),
          Text(
           extent={{72,42},{-14,12}},
            textColor={0,0,0},
            textString=DynamicSelect("", String(TTanTop.T-273.15, format=".1f"))),
          Text(
           extent={{74,-46},{-12,-76}},
            textColor={0,0,0},
            textString=DynamicSelect("", String(TTanBot.T-273.15, format=".1f"))),
          Ellipse(
            extent={{81,-83},{95,-97}},
            lineColor=DynamicSelect({235,235,235},
              if charge then
                {0,255,0}
              else
                {235,235,235}),
            fillColor=DynamicSelect({235,235,235},
              if charge then
                {0,255,0}
              else
                {235,235,235}),
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{-16,2},{16,-2}},
            fillColor={28,108,200},
            fillPattern=FillPattern.Solid,
            pattern=LinePattern.None,
            origin={-72,-76},
            rotation=270),
          Rectangle(
            extent={{-54,2},{54,-2}},
            fillColor={28,108,200},
            fillPattern=FillPattern.Solid,
            pattern=LinePattern.None,
            origin={-20,-90},
            rotation=180),
          Rectangle(
            extent={{-7,2},{7,-2}},
            fillColor={28,108,200},
            fillPattern=FillPattern.Solid,
            pattern=LinePattern.None,
            origin={32,-85},
            rotation=270)}),
          Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  end StorageTankWithExternalHeatExchanger1;

  model DHW
    "A model of a storage tank with external heat exchanger to produce hot water"
    extends Buildings.DHC.Loads.HotWater.BaseClasses.PartialFourPortDHW(
      final allowFlowReversalHea=false);

    parameter Buildings.DHC.Loads.HotWater.Data.GenericDomesticHotWaterWithHeatExchanger
      dat "Performance data"
      annotation (Placement(transformation(extent={{60,80},{80,100}})));

    parameter Real k=0.1 "Proportional gain of circulation pump controller";
    parameter Real Ti=60 "Integrator time constant of circulation pump controller";

    parameter Modelica.Media.Interfaces.Types.Temperature TTan_start=323.15
      "Start value of tank temperature"
      annotation(Dialog(tab="Initialization"));
    final parameter Real eps =
      dat.QHex_flow_nominal / CMin_flow_nominal / ( dat.TDom_nominal + dat.dTHexApp_nominal - dat.TCol_nominal)
      "Heat exchanger effectiveness"
      annotation(Dialog(tab="Advanced"));
    Buildings.Fluid.Movers.Preconfigured.FlowControlled_dp pumHex(
      redeclare package Medium = MediumHea,
      energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
      use_inputFilter=false,
      riseTime=10,
      m_flow_nominal=dat.mHex_flow_nominal,
      dp_nominal=dat.dpHexHea_nominal) "Pump with head as input" annotation (
        Placement(transformation(
          extent={{-10,10},{10,-10}},
          rotation=270,
          origin={-40,0})));

    Modelica.Blocks.Interfaces.RealOutput PEle(unit="W")
      "Electric power required for pumping equipment"
      annotation (Placement(transformation(extent={{100,-10},{120,10}}),
          iconTransformation(extent={{100,-10},{120,10}})));
    Buildings.Fluid.Sensors.TemperatureTwoPort senTemHot(
      redeclare package Medium =  MediumDom,
      final allowFlowReversal=allowFlowReversalDom,
      m_flow_nominal=dat.mDom_flow_nominal)
      "Temperature sensor for hot water supply"
      annotation (Placement(transformation(extent={{20,50},{40,70}})));

    Buildings.Fluid.HeatExchangers.ConstantEffectiveness hex(
      redeclare package Medium1 = MediumDom,
      redeclare package Medium2 = MediumHea,
      final allowFlowReversal1=allowFlowReversalDom,
      m1_flow_nominal=dat.mDom_flow_nominal,
      m2_flow_nominal=dat.mHex_flow_nominal,
      dp1_nominal=dat.dpHexHea_nominal,
      from_dp2=true,
      dp2_nominal=dat.dpHexDom_nominal,
      eps=eps)
      annotation (Placement(transformation(extent={{-60,44},{-40,64}})));
    Buildings.Fluid.Sensors.MassFlowRate senMasFlo(redeclare package Medium = MediumDom)
      "Mass flow rate of domestic hot water"
      annotation (Placement(transformation(extent={{-20,50},{0,70}})));

    Buildings.DHC.Loads.HotWater.BaseClasses.HeatExchangerPumpController conPum(final mDom_flow_nominal=dat.mDom_flow_nominal,
        final dpPum_nominal=dat.dpHexHea_nominal)
      "Controller for pump of heat exchanger"
      annotation (Placement(transformation(extent={{-80,-10},{-60,10}})));

  protected
    parameter Modelica.Units.SI.SpecificHeatCapacity cpHea_default =
      MediumHea.specificHeatCapacityCp(MediumHea.setState_pTX(
        MediumHea.p_default,
        MediumHea.T_default,
        MediumHea.X_default))
      "Specific heat capacity of heating medium at default medium state";
    parameter Modelica.Units.SI.SpecificHeatCapacity cpDom_default =
      MediumDom.specificHeatCapacityCp(MediumDom.setState_pTX(
        MediumDom.p_default,
        MediumDom.T_default,
        MediumDom.X_default))
      "Specific heat capacity of domestic hot water medium at default medium state";
    parameter Modelica.Units.SI.ThermalConductance CMin_flow_nominal =
      min(dat.mHex_flow_nominal*cpHea_default, dat.mDom_flow_nominal*cpDom_default)
      "Minimum heat capacity flow rate";
  initial equation
    assert(eps < 1, "In " + getInstanceName() + ": Heat exchanger effectivness must be below 1, received eps = " + String(eps) + ". Check sizing.");

  equation
    connect(pumHex.P, PEle) annotation (Line(points={{-49,-11},{-50,-11},{-50,-32},
            {86,-32},{86,0},{110,0}},     color={0,0,127}));
    connect(hex.port_b1, senMasFlo.port_a)
      annotation (Line(points={{-40,60},{-20,60}}, color={0,127,255}));
    connect(senMasFlo.port_b, senTemHot.port_a)
      annotation (Line(points={{0,60},{20,60}},    color={0,127,255}));
    connect(senMasFlo.m_flow, conPum.mDom_flow) annotation (Line(points={{-10,71},
            {-10,74},{-86,74},{-86,6},{-82,6}}, color={0,0,127}));
    connect(senTemHot.T, conPum.TDom) annotation (Line(points={{30,71},{30,80},{-88,
            80},{-88,-6},{-81,-6}}, color={0,0,127}));
    connect(conPum.TDomSet, TDomSet) annotation (Line(points={{-81,0},{-92,0},{-92,
            0},{-110,0}}, color={0,0,127}));
    connect(senTemHot.port_b, port_bDom)
      annotation (Line(points={{40,60},{100,60}}, color={0,127,255}));
    connect(conPum.dpPumHex, pumHex.dp_in)
      annotation (Line(points={{-58,0},{-52,0}}, color={0,0,127}));

    connect(port_aDom, hex.port_a1)
      annotation (Line(points={{-100,60},{-60,60}}, color={0,127,255}));
    connect(hex.port_b2, pumHex.port_a) annotation (Line(points={{-60,48},{-72,48},
            {-72,20},{-40,20},{-40,10}}, color={0,127,255}));
    connect(hex.port_a2, port_aHea) annotation (Line(points={{-40,48},{-24,48},{
            -24,24},{126,24},{126,-44},{100,-44},{100,-60}}, color={0,127,255}));
    connect(pumHex.port_b, port_bHea) annotation (Line(points={{-40,-10},{-40,-60},
            {-100,-60}}, color={0,127,255}));
    annotation (
    defaultComponentName="domHotWatTan",
    Documentation(info="<html>
<p>
This model implements a heating hot water tank with external heat exchanger that heats domestic hot water.
</p>
<p>
The storage tank model is described in
<a href=\"modelica://Buildings.Fluid.Storage.StratifiedEnhancedInternalHex\">
Buildings.Fluid.Storage.StratifiedEnhancedInternalHex</a>.
The heat pump and storage tank system should be parameterized altogether using
<a href=\"modelica://Buildings.DHC.Loads.HotWater.Data.GenericDomesticHotWaterWithHeatExchanger\">
Buildings.DHC.Loads.HotWater.Data.GenericDomesticHotWaterWithHeatExchanger</a>.
</p>
<p align=\"center\">
<img alt=\"image\" src=\"modelica://Buildings/Resources/Images/DHC/Loads/HotWater/StorageTankWithExternalHeatExchanger.png\"/>
</p>
<p>
It is based on Fig. 3 in <i>Evaluations of different domestic hot water
preparing methods with ultra-low-temperature district heating</i> by X. Yang,
H. Li, and S. Svendsen at <a href=\"https:/doi.org/10.1016/j.energy.2016.04.109\">
doi.org/10.1016/j.energy.2016.04.109</a>, as well as the
<i>Advanced Energy Design Guide for Multifamily Buildings-Achieving Zero Energy</i>
published by ASHRAE in 2022 at <a href=\"https://www.ashrae.org/technical-resources/aedgs/zero-energy-aedg-free-download\">
https://www.ashrae.org/technical-resources/aedgs/zero-energy-aedg-free-download</a>.
</p>
<p>
For a model that connects this hot water system to a heat pump, see
<a href=\"modelica://Buildings.DHC.ETS.Combined.Subsystems.HeatPumpDHWTank\">
Buildings.DHC.ETS.Combined.Subsystems.HeatPumpDHWTank</a>.
</p>
</html>",   revisions="<html>
<ul>
<li>
October 5, 2023, by Michael Wetter:<br/>
First implementation.
</li>
</ul>
</html>"),  Icon(coordinateSystem(preserveAspectRatio=false), graphics={
          Polygon(
            points={{-140,86},{-140,86}},
            lineColor={95,95,95},
            lineThickness=1,
            fillColor={0,0,0},
            fillPattern=FillPattern.CrossDiag),
          Rectangle(
            extent={{0,-78},{0,-78}},
            lineColor={0,0,0},
            lineThickness=0.5),
          Rectangle(
            extent={{-30,40},{-50,0}},
            lineColor={0,0,0},
            lineThickness=0.5),
          Rectangle(
            extent={{-100,64},{-60,58}},
            fillColor={0,140,72},
            fillPattern=FillPattern.Solid,
            pattern=LinePattern.None),
          Rectangle(
            extent={{-44,64},{100,58}},
            fillColor={102,44,145},
            fillPattern=FillPattern.Solid,
            pattern=LinePattern.None),
          Rectangle(
            extent={{-60,-10},{-44,-16}},
            fillColor={0,140,72},
            fillPattern=FillPattern.Solid,
            pattern=LinePattern.None),
          Rectangle(
            extent={{-40,2},{40,-2}},
            fillColor={0,140,72},
            fillPattern=FillPattern.Solid,
            pattern=LinePattern.None,
            origin={-60,24},
            rotation=90),
          Rectangle(
            extent={{-6,2},{6,-2}},
            fillColor={0,140,72},
            fillPattern=FillPattern.Solid,
            pattern=LinePattern.None,
            origin={-46,-6},
            rotation=90),
          Rectangle(
            extent={{76,-58},{98,-62}},
            fillColor={238,46,47},
            fillPattern=FillPattern.Solid,
            pattern=LinePattern.None),
          Rectangle(
            extent={{-58,2},{58,-2}},
            fillColor={238,46,47},
            fillPattern=FillPattern.Solid,
            pattern=LinePattern.None,
            origin={22,54},
            rotation=180),
          Rectangle(
            extent={{-56.5,2.5},{56.5,-2.5}},
            fillColor={238,46,47},
            fillPattern=FillPattern.Solid,
            pattern=LinePattern.None,
            origin={77.5,-4.5},
            rotation=270),
          Rectangle(
            extent={{-6,2},{6,-2}},
            fillColor={238,46,47},
            fillPattern=FillPattern.Solid,
            pattern=LinePattern.None,
            origin={32,46},
            rotation=270),
          Rectangle(
            extent={{-6,2},{6,-2}},
            fillColor={238,46,47},
            fillPattern=FillPattern.Solid,
            pattern=LinePattern.None,
            origin={-34,46},
            rotation=270),
          Rectangle(
            extent={{-15,2},{15,-2}},
            fillColor={28,108,200},
            fillPattern=FillPattern.Solid,
            pattern=LinePattern.None,
            origin={-85,-60},
            rotation=180),
          Rectangle(
            extent={{-6,2},{6,-2}},
            fillColor={28,108,200},
            fillPattern=FillPattern.Solid,
            pattern=LinePattern.None,
            origin={-34,-50},
            rotation=90),
          Rectangle(
            extent={{-23,2},{23,-2}},
            fillColor={28,108,200},
            fillPattern=FillPattern.Solid,
            pattern=LinePattern.None,
            origin={-34,-23},
            rotation=270),
          Rectangle(
            extent={{-12,2},{12,-2}},
            fillColor={102,44,145},
            fillPattern=FillPattern.Solid,
            pattern=LinePattern.None,
            origin={-44,52},
            rotation=90),
          Text(
            extent={{-116,36},{-66,0}},
            textColor={0,0,127},
            textString="TDomSet"),
          Ellipse(
            extent={{-42,-20},{-26,-34}},
            lineColor={0,0,0},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid),
          Polygon(
            points={{-15,-3},{-7,5},{-7,-11},{-15,-3}},
            lineColor={0,0,0},
            fillColor={0,0,0},
            fillPattern=FillPattern.Solid,
            origin={-37,-19},
            rotation=90),
          Ellipse(
            extent={{81,-83},{95,-97}},
            lineColor=DynamicSelect({235,235,235},
              if charge then
                {0,255,0}
              else
                {235,235,235}),
            fillColor=DynamicSelect({235,235,235},
              if charge then
                {0,255,0}
              else
                {235,235,235}),
            fillPattern=FillPattern.Solid),
        Bitmap(
         imageSource="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",
         extent={{-30,48},{76,-60}}),
          Rectangle(
            extent={{-15,2},{15,-2}},
            fillColor={28,108,200},
            fillPattern=FillPattern.Solid,
            pattern=LinePattern.None,
            origin={-51,-60},
            rotation=180)}),
          Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  end DHW;

  package Validation
    model ExampleStorageTankWithExternalHeatExchanger
      "Example model for storage tank with external heat exchanger"
      extends Modelica.Icons.Example;
      package Medium = Buildings.Media.Water "Medium model";
      parameter Modelica.Units.SI.Temperature TCol = 273.15+10 "Temperature of domestic cold water supply";
      parameter Modelica.Units.SI.MassFlowRate mHea_flow_nominal = datWatHea.QHex_flow_nominal/4200/(55 - 50) "Tank heater water loop nominal mass flow";
      parameter Buildings.DHC.Loads.HotWater.Data.GenericDomesticHotWaterWithHeatExchanger
        datWatHea(VTan=0.1892706, mDom_flow_nominal=6.52944E-06*1000)
        "Data for heat pump water heater with tank"
        annotation (Placement(transformation(extent={{-80,-60},{-60,-40}})));

      Modelica.Blocks.Sources.CombiTimeTable sch(
        tableOnFile=true,
        tableName="tab1",
        fileName=Modelica.Utilities.Files.loadResource(
            "modelica://Buildings/Resources/Data/DHC/Loads/HotWater/DHW_ApartmentMidRise.mos"),
        smoothness=Modelica.Blocks.Types.Smoothness.ContinuousDerivative,
        extrapolation=Modelica.Blocks.Types.Extrapolation.Periodic)
        "Domestic hot water fixture draw fraction schedule"
        annotation (Placement(transformation(extent={{-80,60},{-60,80}})));

      Buildings.Fluid.Sources.Boundary_pT souCol(
        nPorts=2,
        redeclare package Medium = Medium,
        T(displayUnit="degC") = 283.15) "Source of domestic cold water"
        annotation (Placement(transformation(extent={{-10,-10},{10,10}},
            rotation=90,
            origin={-32,-50})));
      Modelica.Blocks.Sources.Constant conTSetMix(k(
          final unit="K",
          displayUnit="degC") = 308.15)
        "Temperature setpoint for mixed water supply to fixture"
        annotation (Placement(transformation(extent={{-80,30},{-60,50}})));
      Buildings.DHC.Loads.HotWater.StorageTankWithExternalHeatExchanger
        domHotWatTan(redeclare package MediumDom = Medium, redeclare package
          MediumHea = Medium,
        dat=datWatHea)        "Storage tank with external heat exchanger"
        annotation (Placement(transformation(extent={{0,20},{20,40}})));
      Buildings.DHC.Loads.HotWater.ThermostaticMixingValve theMixVal(redeclare
          package Medium =                                                                                   Medium,
          mMix_flow_nominal=1.2*datWatHea.mDom_flow_nominal)
        annotation (Placement(transformation(extent={{40,60},{60,80}})));
      Buildings.Controls.OBC.CDL.Conversions.BooleanToReal booToRea(realTrue=
            mHea_flow_nominal)
        annotation (Placement(transformation(extent={{40,-20},{60,0}})));
      Modelica.Blocks.Sources.Constant conTSetHot(k(
          final unit="K",
          displayUnit="degC") = 313.15)
        "Temperature setpoint for hot water supply to fixture"
        annotation (Placement(transformation(extent={{-80,0},{-60,20}})));
      Buildings.Fluid.HeatExchangers.Heater_T hea(
        redeclare package Medium = Medium,
        m_flow_nominal=mHea_flow_nominal,
        dp_nominal=0)
        annotation (Placement(transformation(extent={{40,-50},{60,-30}})));
      Buildings.Fluid.Movers.FlowControlled_m_flow mov(
        redeclare package Medium = Medium,
        energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
        nominalValuesDefineDefaultPressureCurve=true,
        m_flow_nominal=mHea_flow_nominal)
        annotation (Placement(transformation(extent={{70,14},{50,34}})));
      Buildings.Fluid.Sensors.TemperatureTwoPort senTem(
        redeclare package Medium = Medium,
        m_flow_nominal=mHea_flow_nominal,
        tau=0) annotation (Placement(transformation(extent={{10,-50},{30,-30}})));
      Buildings.Controls.OBC.CDL.Reals.AddParameter addPar(p=5) "dT for heater"
        annotation (Placement(transformation(extent={{14,-24},{24,-14}})));
      Buildings.Fluid.Sources.Boundary_pT preRef(
        nPorts=1,
        redeclare package Medium = Medium,
        T(displayUnit="degC")) "Reference pressure" annotation (Placement(
            transformation(
            extent={{-10,-10},{10,10}},
            rotation=90,
            origin={80,-60})));
    equation
      connect(theMixVal.yMixSet, sch.y[1]) annotation (Line(points={{39,78},{-50,78},
              {-50,70},{-59,70}}, color={0,0,127}));
      connect(conTSetMix.y, theMixVal.TMixSet) annotation (Line(points={{-59,40},{
              -10,40},{-10,72},{39,72}},
                                     color={0,0,127}));
      connect(domHotWatTan.port_bDom, theMixVal.port_hot) annotation (Line(points={{
              20,36},{32,36},{32,66},{40,66}}, color={0,127,255}));
      connect(souCol.ports[1], domHotWatTan.port_aDom)
        annotation (Line(points={{-31,-40},{-31,36},{0,36}}, color={0,127,255}));
      connect(souCol.ports[2], theMixVal.port_col)
        annotation (Line(points={{-33,-40},{-33,62},{40,62}}, color={0,127,255}));
      connect(booToRea.u, domHotWatTan.charge) annotation (Line(points={{38,-10},{32,
              -10},{32,21},{22,21}}, color={255,0,255}));
      connect(domHotWatTan.TDomSet, conTSetHot.y) annotation (Line(points={{-1,30},{
              -20,30},{-20,10},{-59,10}}, color={0,0,127}));
      connect(mov.port_b, domHotWatTan.port_aHea)
        annotation (Line(points={{50,24},{20,24}}, color={0,127,255}));
      connect(hea.port_b, mov.port_a) annotation (Line(points={{60,-40},{80,-40},{80,
              24},{70,24}}, color={0,127,255}));
      connect(booToRea.y, mov.m_flow_in) annotation (Line(points={{62,-10},{90,-10},
              {90,46},{60,46},{60,36}}, color={0,0,127}));
      connect(domHotWatTan.port_bHea, senTem.port_a) annotation (Line(points={{0,24},
              {-10,24},{-10,-40},{10,-40}}, color={0,127,255}));
      connect(senTem.port_b, hea.port_a)
        annotation (Line(points={{30,-40},{40,-40}}, color={0,127,255}));
      connect(addPar.y, hea.TSet) annotation (Line(points={{25,-19},{32,-19},{32,-32},
              {38,-32}}, color={0,0,127}));
      connect(senTem.T, addPar.u) annotation (Line(points={{20,-29},{14,-29},{14,-28},
              {6,-28},{6,-19},{13,-19}}, color={0,0,127}));
      connect(hea.port_b, preRef.ports[1])
        annotation (Line(points={{60,-40},{80,-40},{80,-50}}, color={0,127,255}));
      annotation (Diagram(graphics={
            Text(
              extent={{-140,160},{160,120}},
              textString="%name",
              textColor={0,0,255})}),
              __Dymola_Commands(
          file="modelica://Buildings/Resources/Scripts/Dymola/DHC/Loads/HotWater/Examples/StorageTankWithExternalHeatExchanger.mos" "Simulate and plot"),
    Documentation(info="<html>
<p>
Example model of a fresh water station that heats up domestic hot water.
Input is a load profile which is sent to a model that computes the hot and cold water draw.
If the tank needs to be recharged, then tank water is circulated through a heater
with a prescribed temperature lift.
</p>
</html>",     revisions="<html>
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
          Tolerance=1e-06));
    end ExampleStorageTankWithExternalHeatExchanger;

    model TimeSeriesExample
      "Example illustrating the coupling of a building model to heating water and chilled water loops"
      extends Modelica.Icons.Example;
      package Medium1=Buildings.Media.Water
        "Source side medium";
      parameter Modelica.Units.SI.Time perAve=600
        "Period for time averaged variables";
      Buildings.DHC.Loads.BaseClasses.BuildingTimeSeries bui(
        filNam="modelica://Buildings/Resources/Data/DHC/Loads/Examples/SwissResidential_20190916.mos",
        nPorts_aHeaWat=1,
        nPorts_aChiWat=1,
        nPorts_bHeaWat=1,
        nPorts_bChiWat=1)
        "Building"
        annotation (Placement(transformation(extent={{12,-4},{32,16}})));
      Buildings.Fluid.Sources.Boundary_pT sinHeaWat(
        redeclare package Medium=Medium1,
        nPorts=1)
        "Sink for heating water"
        annotation (Placement(transformation(extent={{10,-10},{-10,10}},rotation=0,origin={130,20})));
      Buildings.Fluid.Sources.Boundary_pT sinChiWat(
        redeclare package Medium=Medium1,
        nPorts=1)
        "Sink for chilled water"
        annotation (Placement(transformation(extent={{10,-10},{-10,10}},rotation=0,origin={130,-20})));
      Modelica.Blocks.Sources.RealExpression THeaWatSup(
        y=bui.T_aHeaWat_nominal)
        "Heating water supply temperature"
        annotation (Placement(transformation(extent={{-120,10},{-100,30}})));
      Modelica.Blocks.Sources.RealExpression TChiWatSup(
        y=bui.T_aChiWat_nominal)
        "Chilled water supply temperature"
        annotation (Placement(transformation(extent={{-120,-30},{-100,-10}})));
      Buildings.Fluid.Sources.Boundary_pT supHeaWat(
        redeclare package Medium = Medium1,
        use_T_in=true,
        nPorts=1) "Heating water supply" annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=0,
            origin={-50,20})));
      Buildings.Fluid.Sources.Boundary_pT supChiWat(
        redeclare package Medium = Medium1,
        use_T_in=true,
        nPorts=1) "Chilled water supply" annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=0,
            origin={-50,-20})));
      Modelica.Blocks.Continuous.Integrator EHeaReq(
        y(unit="J"))
        "Time integral of heating load"
        annotation (Placement(transformation(extent={{60,70},{80,90}})));
      Modelica.Blocks.Continuous.Integrator EHeaAct(
        y(unit="J"))
        "Actual energy used for heating"
        annotation (Placement(transformation(extent={{100,70},{120,90}})));
      Modelica.Blocks.Continuous.Integrator ECooReq(
        y(unit="J"))
        "Time integral of cooling load"
        annotation (Placement(transformation(extent={{60,-70},{80,-50}})));
      Modelica.Blocks.Continuous.Integrator ECooAct(
        y(unit="J"))
        "Actual energy used for cooling"
        annotation (Placement(transformation(extent={{100,-70},{120,-50}})));
      Buildings.Controls.OBC.CDL.Reals.MovingAverage QAveHeaReq_flow(y(unit=
              "W"), final delta=perAve) "Time average of heating load"
        annotation (Placement(transformation(extent={{60,110},{80,130}})));
      Buildings.Controls.OBC.CDL.Reals.MovingAverage QAveHeaAct_flow(y(unit=
              "W"), final delta=perAve) "Time average of heating heat flow rate"
        annotation (Placement(transformation(extent={{100,110},{120,130}})));
      Buildings.Controls.OBC.CDL.Reals.MovingAverage QAveCooReq_flow(y(unit=
              "W"), final delta=perAve) "Time average of cooling load"
        annotation (Placement(transformation(extent={{60,-110},{80,-90}})));
      Buildings.Controls.OBC.CDL.Reals.MovingAverage QAveCooAct_flow(y(unit=
              "W"), final delta=perAve) "Time average of cooling heat flow rate"
        annotation (Placement(transformation(extent={{100,-110},{120,-90}})));
    equation
      connect(supHeaWat.T_in,THeaWatSup.y)
        annotation (Line(points={{-62,24},{-80,24},{-80,20},{-99,20}},color={0,0,127}));
      connect(TChiWatSup.y,supChiWat.T_in)
        annotation (Line(points={{-99,-20},{-80,-20},{-80,-16},{-62,-16}},color={0,0,127}));
      connect(supHeaWat.ports[1],bui.ports_aHeaWat[1])
        annotation (Line(points={{-40,20},{0,20},{0,4},{12,4}},color={0,127,255}));
      connect(supChiWat.ports[1],bui.ports_aChiWat[1])
        annotation (Line(points={{-40,-20},{0,-20},{0,0},{12,0}},color={0,127,255}));
      connect(bui.ports_bHeaWat[1],sinHeaWat.ports[1])
        annotation (Line(points={{32,4},{60,4},{60,20},{120,20}},color={0,127,255}));
      connect(sinChiWat.ports[1],bui.ports_bChiWat[1])
        annotation (Line(points={{120,-20},{60,-20},{60,0},{32,0}},color={0,127,255}));
      connect(bui.QHea_flow,EHeaAct.u)
        annotation (Line(points={{32.6667,14.6667},{40,14.6667},{40,60},{90,60},
              {90,80},{98,80}},                                                                  color={0,0,127}));
      connect(bui.QReqHea_flow,EHeaReq.u)
        annotation (Line(points={{28.6667,-5.33333},{28.6667,-8},{36,-8},{36,80},
              {58,80}},                                                                   color={0,0,127}));
      connect(bui.QReqCoo_flow,ECooReq.u)
        annotation (Line(points={{30,-5.33333},{30,-60},{58,-60}},          color={0,0,127}));
      connect(bui.QCoo_flow,ECooAct.u)
        annotation (Line(points={{32.6667,13.3333},{40,13.3333},{40,-40},{90,
              -40},{90,-60},{98,-60}},                                                               color={0,0,127}));
      connect(bui.QReqHea_flow,QAveHeaReq_flow.u)
        annotation (Line(points={{28.6667,-5.33333},{28.6667,-7.90323},{35.9677,
              -7.90323},{35.9677,120},{58,120}},                                                                  color={0,0,127}));
      connect(bui.QHea_flow,QAveHeaAct_flow.u)
        annotation (Line(points={{32.6667,14.6667},{40,14.6667},{40,60},{90,60},
              {90,120},{98,120}},                                                                  color={0,0,127}));
      connect(bui.QReqCoo_flow,QAveCooReq_flow.u)
        annotation (Line(points={{30,-5.33333},{28.6316,-5.33333},{28.6316,-100},{
              58,-100}},                                                                                       color={0,0,127}));
      connect(bui.QCoo_flow,QAveCooAct_flow.u)
        annotation (Line(points={{32.6667,13.3333},{40,13.3333},{40,-40},{90,
              -40},{90,-100},{98,-100}},                                                               color={0,0,127}));
      annotation (
        experiment(
          StopTime=604800,
          Tolerance=1e-06),
        Documentation(
          info="<html>
<p>
This example illustrates the use of
<a href=\"modelica://Buildings.DHC.Loads.BaseClasses.PartialBuilding\">
Buildings.DHC.Loads.BaseClasses.PartialBuilding</a>,
<a href=\"modelica://Buildings.DHC.Loads.BaseClasses.PartialTerminalUnit\">
Buildings.DHC.Loads.BaseClasses.PartialTerminalUnit</a>
and
<a href=\"modelica://Buildings.DHC.Loads.BaseClasses.FlowDistribution\">
Buildings.DHC.Loads.BaseClasses.FlowDistribution</a>
in a configuration with
</p>
<ul>
<li>
space heating and cooling loads provided as time series, and
</li>
<li>
secondary pumps.
</li>
</ul>
</html>", revisions="<html>
<ul>
<li>
November 21, 2022, by David Blum:<br/>
Change <code>bui.facMulHea</code> and <code>bui.facMulCoo</code> to be default.<br/>
This is for
<a href=\"https://github.com/lbl-srg/modelica-buildings/issues/2302\">
issue 2302</a>.
</li>
<li>
February 21, 2020, by Antoine Gautier:<br/>
First implementation.
</li>
</ul>
</html>"),
        Diagram(
          coordinateSystem(
            preserveAspectRatio=false,
            extent={{-160,-140},{160,140}})),
        __Dymola_Commands(
          file="modelica://Buildings/Resources/Scripts/Dymola/DHC/Loads/BaseClasses/Examples/CouplingTimeSeries.mos" "Simulate and plot"));
    end TimeSeriesExample;

    model OnlyHeatingTimeSeriesSingleLoop
      "Example illustrating the coupling of a building model to heating water or chilled water loops"
      extends Modelica.Icons.Example;
      package Medium1=Buildings.Media.Water
        "Source side medium";
      parameter Modelica.Units.SI.Time perAve=600
        "Period for time averaged variables";
      Buildings.DHC.Loads.BaseClasses.BuildingTimeSeries buiCoo(
        have_heaWat=false,
        filNam="modelica://Buildings/Resources/Data/DHC/Loads/Examples/SwissResidential_20190916.mos",
        nPorts_aChiWat=1,
        nPorts_bChiWat=1)
        "Building wint cooling only"
        annotation (Placement(transformation(extent={{-10,100},{10,120}})));
      Buildings.Fluid.Sources.Boundary_pT sinChiWat(
        redeclare package Medium=Medium1,
        nPorts=1)
        "Sink for chilled water"
        annotation (Placement(transformation(extent={{10,-10},{-10,10}},rotation=0,origin={110,104})));
      Modelica.Blocks.Sources.RealExpression TChiWatSup(
        y=buiCoo.T_aChiWat_nominal)
        "Chilled water supply temperature"
        annotation (Placement(transformation(extent={{-140,98},{-120,118}})));
      Buildings.Fluid.Sources.Boundary_pT supChiWat(
        redeclare package Medium = Medium1,
        use_T_in=true,
        nPorts=1) "Chilled water supply" annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=0,
            origin={-70,104})));
      Modelica.Blocks.Continuous.Integrator ECooReq(
        y(unit="J"))
        "Time integral of cooling load"
        annotation (Placement(transformation(extent={{40,60},{60,80}})));
      Buildings.Controls.OBC.CDL.Reals.MovingAverage QAveCooReq_flow(y(unit=
              "W"), final delta=perAve) "Time average of cooling load"
        annotation (Placement(transformation(extent={{40,20},{60,40}})));
      Buildings.Controls.OBC.CDL.Reals.MovingAverage QAveCooAct_flow(y(unit=
              "W"), final delta=perAve) "Time average of cooling heat flow rate"
        annotation (Placement(transformation(extent={{80,20},{100,40}})));
      Modelica.Blocks.Continuous.Integrator ECooAct(
        y(unit="J"))
        "Actual energy used for cooling"
        annotation (Placement(transformation(extent={{80,60},{100,80}})));
      Buildings.DHC.Loads.BaseClasses.BuildingTimeSeries buiHea(
        have_chiWat=false,
        filNam="modelica://Buildings/Resources/Data/DHC/Loads/Examples/SwissResidential_20190916.mos",
        nPorts_aChiWat=1,
        nPorts_bChiWat=1,
        nPorts_aHeaWat=1,
        nPorts_bHeaWat=1)
        "Building with heating only"
        annotation (Placement(transformation(extent={{-10,-20},{10,0}})));
      Modelica.Blocks.Sources.RealExpression THeaWatSup(
        y=buiHea.T_aHeaWat_nominal)
        "Heating water supply temperature"
        annotation (Placement(transformation(extent={{-140,-18},{-120,2}})));
      Buildings.Fluid.Sources.Boundary_pT supHeaWat(
        redeclare package Medium = Medium1,
        use_T_in=true,
        nPorts=1) "Heating water supply" annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=0,
            origin={-70,-12})));
      Buildings.Fluid.Sources.Boundary_pT sinHeaWat(redeclare package Medium =
            Medium1, nPorts=1) "Sink for heating water" annotation (Placement(
            transformation(
            extent={{10,-10},{-10,10}},
            rotation=0,
            origin={110,-12})));
      Buildings.Controls.OBC.CDL.Reals.MovingAverage QAveHeaReq_flow(y(unit=
              "W"), final delta=perAve) "Time average of heating load"
        annotation (Placement(transformation(extent={{40,-110},{60,-90}})));
      Modelica.Blocks.Continuous.Integrator EHeaReq(
        y(unit="J"))
        "Time integral of heating load"
        annotation (Placement(transformation(extent={{40,-70},{60,-50}})));
      Modelica.Blocks.Continuous.Integrator EHeaAct(
        y(unit="J"))
        "Actual energy used for heating"
        annotation (Placement(transformation(extent={{80,-70},{100,-50}})));
      Buildings.Controls.OBC.CDL.Reals.MovingAverage QAveHeaAct_flow(y(unit=
              "W"), final delta=perAve) "Time average of heating heat flow rate"
        annotation (Placement(transformation(extent={{80,-110},{100,-90}})));
    equation
      connect(TChiWatSup.y,supChiWat.T_in)
        annotation (Line(points={{-119,108},{-82,108}},color={0,0,127}));
      connect(supChiWat.ports[1],buiCoo.ports_aChiWat[1])
        annotation (Line(points={{-60,104},{-10,104}},color={0,127,255}));
      connect(sinChiWat.ports[1],buiCoo.ports_bChiWat[1])
        annotation (Line(points={{100,104},{10,104}},color={0,127,255}));
      connect(buiCoo.QReqCoo_flow,ECooReq.u)
        annotation (Line(points={{8,98.6667},{8,70},{38,70}},            color={0,0,127}));
      connect(buiCoo.QReqCoo_flow,QAveCooReq_flow.u)
        annotation (Line(points={{8,98.6667},{8,30},{38,30}},            color={0,0,127}));
      connect(buiCoo.QCoo_flow,ECooAct.u)
        annotation (Line(points={{10.6667,117.333},{70,117.333},{70,70},{78,70}},color={0,0,127}));
      connect(buiCoo.QCoo_flow,QAveCooAct_flow.u)
        annotation (Line(points={{10.6667,117.333},{70,117.333},{70,30},{78,30}},color={0,0,127}));
      connect(THeaWatSup.y,supHeaWat.T_in)
        annotation (Line(points={{-119,-8},{-82,-8}},color={0,0,127}));
      connect(supHeaWat.ports[1],buiHea.ports_aHeaWat[1])
        annotation (Line(points={{-60,-12},{-10,-12}},color={0,127,255}));
      connect(buiHea.ports_bHeaWat[1],sinHeaWat.ports[1])
        annotation (Line(points={{10,-12},{100,-12}},color={0,127,255}));
      connect(buiHea.QReqHea_flow,EHeaReq.u)
        annotation (Line(points={{6.66667,-21.3333},{6.66667,-60},{38,-60}},color={0,0,127}));
      connect(buiHea.QReqHea_flow,QAveHeaReq_flow.u)
        annotation (Line(points={{6.66667,-21.3333},{6.66667,-100},{38,-100}},color={0,0,127}));
      connect(buiHea.QHea_flow,EHeaAct.u)
        annotation (Line(points={{10.6667,-1.33333},{70,-1.33333},{70,-60},{78,
              -60}},                                                                 color={0,0,127}));
      connect(buiHea.QHea_flow,QAveHeaAct_flow.u)
        annotation (Line(points={{10.6667,-1.33333},{70,-1.33333},{70,-100},{78,
              -100}},                                                                  color={0,0,127}));
      annotation (
        experiment(
          StopTime=604800,
          Tolerance=1e-06),
        Documentation(
          info="<html>
<p>
This example illustrates the use of
<a href=\"modelica://Buildings.DHC.Loads.BaseClasses.PartialBuilding\">
Buildings.DHC.Loads.BaseClasses.PartialBuilding</a>,
<a href=\"modelica://Buildings.DHC.Loads.BaseClasses.PartialTerminalUnit\">
Buildings.DHC.Loads.BaseClasses.PartialTerminalUnit</a>
and
<a href=\"modelica://Buildings.DHC.Loads.BaseClasses.FlowDistribution\">
Buildings.DHC.Loads.BaseClasses.FlowDistribution</a>
in a configuration with
</p>
<ul>
<li>
a single connection with a heating water distribution system, see
component <code>buiHea</code> (resp. with a chilled water distribution
system, see component <code>buiCoo</code>),
</li>
<li>
space heating and cooling loads provided as time series, and
</li>
<li>
secondary pumps.
</li>
</ul>
</html>", revisions="<html>
<ul>
<li>
November 21, 2022, by David Blum:<br/>
Change <code>buiHea.facMulHea</code> and <code>buiCoo.facMulCoo</code> to be default.<br/>
This is for
<a href=\"https://github.com/lbl-srg/modelica-buildings/issues/2302\">
issue 2302</a>.
</li>
<li>
September 18, 2020, by Jianjun Hu:<br/>
First implementation.
</li>
</ul>
</html>"),
        Diagram(
          coordinateSystem(
            preserveAspectRatio=false,
            extent={{-160,-140},{160,140}})),
        __Dymola_Commands(
          file="modelica://Buildings/Resources/Scripts/Dymola/DHC/Loads/BaseClasses/Examples/CouplingTimeSeriesSingleLoop.mos" "Simulate and plot"));
    end OnlyHeatingTimeSeriesSingleLoop;
  end Validation;
end Consumers;
