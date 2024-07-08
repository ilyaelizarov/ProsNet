within ProsNet;
package BidirectionalSubstation "Models that are currently under development"
  package New_Substation

    model GeneralHouse

      extends
        ProsNet.Prosumers.SecondarySides.BaseClasses.ControlVolumeDynParam;

      Modelica.Fluid.Interfaces.FluidPort_a port_hot(redeclare package Medium =
            Media.Water)
        annotation (Placement(transformation(extent={{-70,-110},{-50,-90}})));
      Modelica.Fluid.Interfaces.FluidPort_b port_cold(redeclare package Medium =
            Media.Water)
        annotation (Placement(transformation(extent={{50,-110},{70,-90}})));
      Modelica.Blocks.Interfaces.RealOutput m_dot_sec_is "kg/s" annotation (
          Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=-90,
            origin={0,-98})));
      Modelica.Blocks.Interfaces.RealOutput T_sec_hot annotation (Placement(
            transformation(
            extent={{-10,-10},{10,10}},
            rotation=-90,
            origin={-80,-100})));
      Modelica.Blocks.Interfaces.RealOutput T_sec_cold annotation (Placement(
            transformation(
            extent={{-10,-10},{10,10}},
            rotation=-90,
            origin={80,-100})));
      Modelica.Fluid.Sensors.TemperatureTwoPort T_sens_hot(redeclare package
          Medium =
            Media.Water) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=90,
            origin={-60,-50})));
      Modelica.Fluid.Sensors.TemperatureTwoPort T_sens_cold(redeclare package
          Medium = Media.Water) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=-90,
            origin={60,-50})));
      Modelica.Fluid.Sensors.MassFlowRate massFlowRate(redeclare package Medium =
                   Media.Water) annotation (Placement(
            transformation(
            extent={{-10,10},{10,-10}},
            rotation=90,
            origin={-58,-16})));
      Prosumers.SF1 sF1_1
        annotation (Placement(transformation(extent={{-28,-24},{56,38}})));
    equation
      connect(T_sens_hot.port_a, port_hot)
        annotation (Line(points={{-60,-60},{-60,-100}}, color={0,127,255}));
      connect(T_sens_hot.T, T_sec_hot) annotation (Line(points={{-71,-50},{-80,-50},
              {-80,-100}}, color={0,0,127}));
      connect(T_sens_cold.port_b, port_cold)
        annotation (Line(points={{60,-60},{60,-100}}, color={0,127,255}));
      connect(T_sens_cold.T, T_sec_cold)
        annotation (Line(points={{71,-50},{80,-50},{80,-100}}, color={0,0,127}));
      connect(massFlowRate.port_a, T_sens_hot.port_b)
        annotation (Line(points={{-58,-26},{-58,-34},{-60,-34},{-60,-40}},
                                                       color={0,127,255}));
      connect(massFlowRate.m_flow, m_dot_sec_is) annotation (Line(points={{-47,-16},
              {-34,-16},{-34,-84},{0,-84},{0,-98}}, color={0,0,127}));
      connect(massFlowRate.port_b, sF1_1.port_a1) annotation (Line(points={{-58,
              -6},{-58,53.4},{20.8,53.4}}, color={0,127,255}));
      connect(T_sens_cold.port_a, sF1_1.port_b1) annotation (Line(points={{60,
              -40},{60,52.8},{25.2,52.8}}, color={0,127,255}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
            Rectangle(
              extent={{-100,40},{100,-100}},
              lineColor={0,0,0},
              lineThickness=1),
            Polygon(
              points={{100,40},{8,100},{-8,100},{-100,40},{100,40}},
              lineColor={0,0,0},
              lineThickness=1)}),
          Diagram(coordinateSystem(preserveAspectRatio=false)));
    end GeneralHouse;

    model HTS

      replaceable package Medium_prim = ProsNet.Media.Water;
      replaceable package Medium_sec = ProsNet.Media.Water;

      extends ProsNet.Prosumers.BaseClasses.PrimarySideParameters;
      extends ProsNet.Prosumers.SecondarySides.BaseClasses.PumpsPairDynParam;
      extends
        ProsNet.Prosumers.SecondarySides.BaseClasses.ControlVolumeDynParam;

      Modelica.Blocks.Interfaces.RealVectorInput contr_vars_real[6]
    annotation (Placement(transformation(extent={{-222,-20},{-182,20}})));

      Modelica.Blocks.Interfaces.RealVectorOutput states[8]
    annotation (Placement(transformation(extent={{180,-20},{220,20}})));


      Real pi
    annotation (Placement(transformation(extent={{-180,0},{-140,40}})));
      Real mu
    annotation (Placement(transformation(extent={{-180,-40},{-140,0}})));
      Real u_set_prim
    annotation (Placement(transformation(extent={{-180,-80},{-140,-40}})));
      Real kappa_set
    annotation (Placement(transformation(extent={{-180,-120},{-140,-80}})));

    Real T_prim_hot(unit="K", displayUnit="degC")                                 "K"
                                                   annotation (Placement(
        transformation(
        extent={{-20,-20},{20,20}},
        rotation=180,
        origin={160,60})));

     Real T_prim_cold(unit="K", displayUnit="degC") "K"
                                                    annotation (Placement(
        transformation(
        extent={{-20,-20},{20,20}},
        rotation=180,
        origin={160,20})));

     Real V_dot_prim(unit="l/min", displayUnit="l/min") "l/min"
                                                   annotation (Placement(
        transformation(
        extent={{-20,-20},{20,20}},
        rotation=180,
        origin={160,-20})));

     Real V_dot_sec(unit="l/min", displayUnit="l/min") "l/min"
                                                  annotation (Placement(
        transformation(
        extent={{20,-20},{-20,20}},
        rotation=0,
        origin={160,-60})));

      Real Q_dot_is(unit="kW", displayUnit="kW")
    "kW" annotation (Placement(transformation(
        extent={{20,-20},{-20,20}},
        rotation=0,
        origin={160,-100})));

      Real Delta_p_prim(unit="Pa", displayUnit="bar") annotation (
      Placement(transformation(
        extent={{-20,-20},{20,20}},
        rotation=180,
        origin={160,-140})));

      Real cp_prim;

      Fluid.HeatExchangers.LiquidToLiquid heat_exchanger(
    redeclare package Medium2 = Medium_sec,
    redeclare package Medium1 = Medium_prim,
    m1_flow_nominal=m_flow_nominal_1,
    m2_flow_nominal=m_flow_nominal_2,
    show_T=true,
    dp1_nominal=dp1_nominal,
    dp2_nominal=dp2_nominal,
    Q_flow_nominal=Q_flow_nominal,
    T_a1_nominal=338.15,
    T_a2_nominal=313.15)
    annotation (Placement(transformation(extent={{30,12},{50,-8}})));

      Fluid.FixedResistances.CheckValve cheVa_sec_cons(
    m_flow_nominal=m_flow_nominal_2,
    redeclare final package Medium = Medium_sec,
    final CvData=ProsNet.Fluid.Types.CvTypes.Kv,
    final Kv=Kv_cheVal,
    final l=l_cheVal) annotation (Placement(transformation(
        extent={{-10,10},{10,-10}},
        rotation=270,
        origin={60,42})));
      Fluid.FixedResistances.CheckValve cheVal_sec_prod(
    m_flow_nominal=m_flow_nominal_2,
    redeclare final package Medium = Medium_sec,
    final CvData=ProsNet.Fluid.Types.CvTypes.Kv,
    final Kv=Kv_cheVal,
    final l=l_cheVal) annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=270,
        origin={100,70})));
      Fluid.Valves.TwoWayEqualPercentage valve_prim_cons(
    m_flow_nominal=m_flow_nominal_1,
    redeclare final package Medium = Medium_prim,
    final CvData=ProsNet.Fluid.Types.CvTypes.Kv,
    final Kv=Kv_conVal,
    final use_inputFilter=use_inputFilter_conVal,
    final riseTime=riseTime_conVal,
    final init=init_conVal,
    final y_start=y_start_conVal,
    final l=l_conVal) annotation (Placement(transformation(
        extent={{10,10},{-10,-10}},
        rotation=-90,
        origin={60,-40})));
      Fluid.Pumps.SpeedControlled_y pump_prim_prod(
    redeclare final package Medium = Medium_prim,
    final energyDynamics=energyDynamics_feedPump,
    T_start=313.15,
    final tau=tau_feedPump,
    final per=feedinPer,
    final use_inputFilter=use_inputFilter_feedPump,
    final riseTime=riseTime_feedPump,
    final init=init_feedPump,
    final y_start=y_start_feedPump) annotation (Placement(transformation(
        extent={{10,10},{-10,-10}},
        rotation=-90,
        origin={100,-70})));
      Fluid.FixedResistances.CheckValve cheVal_prim_prod(
    m_flow_nominal=m_flow_nominal_1,
    redeclare final package Medium = Medium_prim,
    final CvData=ProsNet.Fluid.Types.CvTypes.Kv,
    final Kv=Kv_cheVal,
    final l=l_cheVal) annotation (Placement(transformation(
        extent={{10,10},{-10,-10}},
        rotation=-90,
        origin={100,-40})));
      Fluid.FixedResistances.CheckValve cheVal_prim_cons(
    m_flow_nominal=m_flow_nominal_1,
    redeclare final package Medium = Medium_prim,
    final CvData=ProsNet.Fluid.Types.CvTypes.Kv,
    final Kv=Kv_cheVal,
    final l=l_cheVal) annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=90,
        origin={62,-70})));
      Modelica.Fluid.Interfaces.FluidPort_b cold_prim(redeclare final package
          Medium =
            Medium_prim)
    annotation (Placement(transformation(extent={{130,-190},{150,-170}})));
      Modelica.Fluid.Interfaces.FluidPort_a hot_prim(redeclare final package
          Medium =
            Medium_prim)
    annotation (Placement(transformation(extent={{-150,-192},{-130,-172}})));
      Conversion conversion
    annotation (Placement(transformation(extent={{-100,-4},{-52,66}})));
      Modelica.Fluid.Sensors.MassFlowRate m_dot_sens_prim(
    redeclare package Medium = Medium_prim, allowFlowReversal=true)
                                            annotation (Placement(
        transformation(
        extent={{10,-10},{-10,10}},
        rotation=90,
        origin={-20,-72})));
      Modelica.Fluid.Sensors.TemperatureTwoPort T_sens_prim_hot(
    redeclare package Medium = Medium_prim, allowFlowReversal=true) annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-20,-110})));
      Modelica.Fluid.Sensors.TemperatureTwoPort T_sens_prim_cold(
    redeclare package Medium = Medium_prim, allowFlowReversal=true)
                                            annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={80,-110})));
      Fluid.Sources.Boundary_pT bou(
        redeclare package Medium = Medium_sec,
    T=313.15,
      nPorts=1)
    annotation (Placement(transformation(extent={{128,118},{108,138}})));

      Fluid.Sensors.RelativePressure pressureDifference(redeclare package
          Medium =
            Medium_prim)
    annotation (Placement(transformation(extent={{4,-164},{24,-144}})));

      Fluid.Pipes.InsulatedPipe_plug pipe_prim_hot(
    T_amb=ambient_temperature,
    R_ins=R_ins_transferpipe,
    length=length_transfer_pipe_tot/2,
        diameter=d_transferpipe/2,
    zeta=zeta_transferstation/2)  annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-20,-138})));
      Fluid.Pipes.InsulatedPipe_plug pipe_prim_cold(
    T_amb=ambient_temperature,
    R_ins=R_ins_transferpipe,
    length=length_transfer_pipe_tot/2,
        diameter=d_transferpipe/2,
    zeta=zeta_transferstation/2)  annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={80,-140})));
      Fluid.Pumps.SpeedControlled_y pump_sec_prod(
        redeclare final package Medium = Medium_prim,
        final energyDynamics=energyDynamics_feedPump,
        T_start=313.15,
        final tau=tau_feedPump,
        final per=feedinPer,
        final use_inputFilter=use_inputFilter_feedPump,
        final riseTime=riseTime_feedPump,
        final init=init_feedPump,
        final y_start=y_start_feedPump) annotation (Placement(transformation(
            extent={{10,10},{-10,-10}},
            rotation=-90,
            origin={100,28})));
      Fluid.Pumps.SpeedControlled_y pump_sec_cons(
        redeclare final package Medium = Medium_prim,
        final energyDynamics=energyDynamics_feedPump,
        T_start=313.15,
        final tau=tau_feedPump,
        final per=feedinPer,
        final use_inputFilter=use_inputFilter_feedPump,
        final riseTime=riseTime_feedPump,
        final init=init_feedPump,
        final y_start=y_start_feedPump) annotation (Placement(transformation(
            extent={{10,10},{-10,-10}},
            rotation=90,
            origin={60,74})));
      Real u_set_sec_cons
    annotation (Placement(transformation(extent={{-180,40},{-140,80}})));
      Real u_set_sec_prod
    annotation (Placement(transformation(extent={{-180,80},{-140,120}})));
      Modelica.Fluid.Sensors.TemperatureTwoPort T_sens_sec_hot(redeclare
          package Medium =
                   Medium_prim, allowFlowReversal=true) annotation (Placement(
            transformation(
            extent={{-10,-10},{10,10}},
            rotation=90,
            origin={-40,152})));
      Modelica.Fluid.Sensors.TemperatureTwoPort T_sens_sec_cold(redeclare
          package Medium =
                   Medium_prim, allowFlowReversal=true) annotation (Placement(
            transformation(
            extent={{-10,-10},{10,10}},
            rotation=-90,
            origin={52,106})));
      Modelica.Fluid.Sensors.MassFlowRate massFlowRate(redeclare package Medium =
            Media.Water)        annotation (Placement(
            transformation(
            extent={{-10,10},{10,-10}},
            rotation=270,
            origin={-14,116})));
      Modelica.Fluid.Interfaces.FluidPort_a port_a1
        "Fluid connector a (positive design flow direction is from port_a to port_b)"
        annotation (Placement(transformation(extent={{82,160},{102,180}})));
      Modelica.Fluid.Interfaces.FluidPort_b port_b1
        "Fluid connector b (positive design flow direction is from port_a to port_b)"
        annotation (Placement(transformation(extent={{-100,160},{-80,180}})));
    equation


      connect(
          contr_vars_real[3], conversion.pi);
      connect(
          contr_vars_real[4], conversion.mu);
      connect(contr_vars_real[5], conversion.u_set_prim);
      connect(
          contr_vars_real[6], conversion.kappa_set);

      u_set_sec_cons = contr_vars_real[1];
      u_set_sec_prod = contr_vars_real[2];
      pi = contr_vars_real[3];
      mu = contr_vars_real[4];
      u_set_prim = contr_vars_real[5];
      kappa_set = contr_vars_real[6];

      cp_prim = 4200;
      states[7] = m_dot_sens_prim.port_a.m_flow*cp_prim*(T_sens_prim_hot.T -
        T_sens_prim_cold.T)/1000;

      connect(
          heat_exchanger.port_b1, valve_prim_cons.port_b)
    annotation (Line(points={{50,-4},{60,-4},{60,-30}}, color={0,127,255}));
      connect(
          cheVal_prim_prod.port_b,heat_exchanger. port_b1)
    annotation (Line(points={{100,-30},{100,-4},{50,-4}},
                                                        color={0,127,255}));
      connect(
          cheVa_sec_cons.port_b,heat_exchanger. port_a2)
    annotation (Line(points={{60,32},{60,8},{50,8}}, color={0,127,255}));
      connect(
          valve_prim_cons.port_a, cheVal_prim_cons.port_a)
    annotation (Line(points={{60,-50},{60,-60},{62,-60}},
                                                 color={0,127,255}));
      connect(
          cheVal_prim_prod.port_a, pump_prim_prod.port_b)
    annotation (Line(points={{100,-50},{100,-60}},
                                                 color={0,127,255}));
      connect(
          conversion.pump_contr, pump_prim_prod.y) annotation (Line(points={{-52,
              19.3333},{-38,19.3333},{-38,20},{-24,20},{-24,-24},{82,-24},{82,
              -70},{88,-70}},
                    color={0,0,127}));
      connect(
          conversion.valve_contr, valve_prim_cons.y) annotation (Line(points={{-52,
              11.5556},{-42,11.5556},{-42,12},{-32,12},{-32,-40},{48,-40}},
                                                                       color={0,
          0,127}));

      connect(
          m_dot_sens_prim.m_flow, conversion.m_dot_prim_is);
      connect(
          cheVal_prim_cons.port_b, T_sens_prim_cold.port_a) annotation (Line(
        points={{62,-80},{62,-90},{80,-90},{80,-100}}, color={0,127,255}));
      connect(
          pump_prim_prod.port_a, T_sens_prim_cold.port_a) annotation (Line(
        points={{100,-80},{100,-90},{80,-90},{80,-100}},
                                                       color={0,127,255}));

      connect(
          bou.ports[1], cheVal_sec_prod.port_b)
    annotation (Line(points={{108,128},{100,128},{100,80}},
                                                         color={0,127,255}));
      connect(
          hot_prim, pressureDifference.port_a) annotation (Line(points={{-140,-182},
          {-140,-154},{4,-154}},  color={0,127,255}));
      connect(
          pressureDifference.port_b, cold_prim) annotation (Line(points={{24,-154},
          {68,-154},{68,-166},{140,-166},{140,-180}},
                                color={0,127,255}));
      connect(
          heat_exchanger.port_a1, m_dot_sens_prim.port_a)
    annotation (Line(points={{30,-4},{-20,-4},{-20,-62}},  color={0,127,255}));
      connect(
          m_dot_sens_prim.port_b, T_sens_prim_hot.port_b)
    annotation (Line(points={{-20,-82},{-20,-100}}, color={0,127,255}));

      connect(
          T_sens_prim_hot.T, states[1]);
      connect(
          T_sens_prim_cold.T, states[2]);
      connect(
           T_sens_sec_hot.T, states[3]);
      connect(
          T_sens_sec_cold.T, states[4]);
      connect(
          conversion.V_dot_prim_is, states[5]);
      connect(
          conversion.V_dot_sec_is, states[6]);
      connect(
          pressureDifference.p_rel, states[8]);

      T_prim_hot = T_sens_prim_hot.T;
      T_prim_cold = T_sens_prim_cold.T;
      V_dot_prim = conversion.V_dot_prim_is;
      V_dot_sec = conversion.V_dot_sec_is;
      Q_dot_is = states[7];
      Delta_p_prim = pressureDifference.p_rel;

      connect(
          pipe_prim_hot.port_b, T_sens_prim_hot.port_a)
    annotation (Line(points={{-20,-128},{-20,-120}}, color={0,127,255}));
      connect(
          pipe_prim_hot.port_a, hot_prim) annotation (Line(points={{-20,-148},{-20,-154},
          {-140,-154},{-140,-182}}, color={0,127,255}));
      connect(
          T_sens_prim_cold.port_b, pipe_prim_cold.port_a)
    annotation (Line(points={{80,-120},{80,-130}}, color={0,127,255}));
      connect(
          pipe_prim_cold.port_b, cold_prim) annotation (Line(points={{80,-150},{80,-166},
          {140,-166},{140,-180}}, color={0,127,255}));
      connect(pump_sec_prod.port_a, heat_exchanger.port_a2)
        annotation (Line(points={{100,18},{100,8},{50,8}}, color={0,127,255}));
      connect(pump_sec_prod.port_b, cheVal_sec_prod.port_a)
        annotation (Line(points={{100,38},{100,60}}, color={0,127,255}));
      connect(conversion.u_set_sec_cons, contr_vars_real[1]) annotation (Line(
            points={{-99.04,57.8333},{-134,57.8333},{-134,-8.33333},{-202,
              -8.33333}},
            color={0,0,127}));
      connect(conversion.u_set_sec_prod, contr_vars_real[2]) annotation (Line(
            points={{-99.04,50.0556},{-134,50.0556},{-134,-5},{-202,-5}}, color={0,0,
              127}));
      connect(conversion.pump_y2, pump_sec_cons.y) annotation (Line(points={{
              -50.08,57.4444},{38,57.4444},{38,74},{72,74}}, color={0,0,127}));
      connect(conversion.pump_y1, pump_sec_prod.y) annotation (Line(points={{
              -50.56,62.1111},{2,62.1111},{2,56},{80,56},{80,28},{88,28}},
            color={0,0,127}));
      connect(pump_sec_cons.port_b, cheVa_sec_cons.port_a)
        annotation (Line(points={{60,64},{60,52}}, color={0,127,255}));
      connect(T_sens_sec_cold.port_b, pump_sec_cons.port_a) annotation (Line(
            points={{52,96},{52,90},{60,90},{60,84}}, color={0,127,255}));
      connect(T_sens_sec_cold.port_b, cheVal_sec_prod.port_b) annotation (Line(
            points={{52,96},{52,90},{100,90},{100,80}}, color={0,127,255}));
      connect(massFlowRate.port_a, T_sens_sec_hot.port_a) annotation (Line(
            points={{-14,126},{-14,136},{-40,136},{-40,142}}, color={0,127,255}));
      connect(massFlowRate.port_b, heat_exchanger.port_b2) annotation (Line(
            points={{-14,106},{-14,8},{30,8}}, color={0,127,255}));
      connect(massFlowRate.m_flow, conversion.m_dot_sec_is) annotation (Line(
            points={{-25,116},{-30,116},{-30,52},{-52,52}},           color={0,
              0,127}));
      connect(T_sens_sec_cold.port_a, port_a1) annotation (Line(points={{52,116},
              {72,116},{72,170},{92,170}}, color={0,127,255}));
      connect(T_sens_sec_hot.port_b, port_b1) annotation (Line(points={{-40,162},
              {-66,162},{-66,170},{-90,170}}, color={0,127,255}));
      annotation (Diagram(coordinateSystem(extent={{-200,-180},{200,180}})), Icon(
        coordinateSystem(extent={{-200,-180},{200,180}}), graphics={
        Line(
          points={{-140,-98},{-140,-170}},
          color={238,46,47},
          thickness=0.5),
        Line(
          points={{138,-100},{138,-168}},
          color={28,108,200},
          thickness=0.5),
        Rectangle(
          extent={{-38,30},{48,-48}},
          lineColor={0,0,0},
          lineThickness=1),
        Bitmap(extent={{-106,88},{108,176}}, fileName=
              "modelica://ProsNet/../../../../Downloads/noun-home-121812.svg"),
        Line(
          points={{32,86},{32,28}},
          color={28,108,200},
          thickness=0.5),
        Line(
          points={{-22,86},{-22,28}},
          color={238,46,47},
          thickness=0.5),
        Line(
          points={{138,-100},{38,-100}},
          color={28,108,200},
          thickness=0.5),
        Line(
          points={{-142,-100},{-24,-100}},
          color={238,46,47},
          thickness=0.5),
        Line(
          points={{-24,-52},{-24,-102}},
          color={238,46,47},
          thickness=0.5),
        Line(
          points={{36,-50},{36,-102}},
          color={28,108,200},
          thickness=0.5),
        Rectangle(extent={{-200,180},{200,-180}}, lineColor={0,0,0})}));
    end HTS;

    model Conversion

    package Medium = Modelica.Media.Water.ConstantPropertyLiquidWater;

      Modelica.Blocks.Sources.Constant factor1(k=(1/60000)*Medium.d_const)
        annotation (Placement(transformation(extent={{-52,40},{-32,60}})));
      Modelica.Blocks.Math.Product volume2mass_flow
        annotation (Placement(transformation(extent={{-10,22},{10,42}})));
      Controls.SecondaryFlowControl secFlowCon
        annotation (Placement(transformation(extent={{30,4},{50,24}})));
      Controls.PrimaryFlowControl priFlowCon
        annotation (Placement(transformation(extent={{-8,-26},{12,-46}})));
      Controls.Linearizer         lin(redeclare final
          Controls.Data.Linearizer.EqualPercentage cha)
       annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=0,
            origin={-30,-70})));
      Modelica.Blocks.Interfaces.RealInput pi "{0;1}"
        annotation (Placement(transformation(extent={{-120,-20},{-80,20}})));
      Modelica.Blocks.Interfaces.RealInput mu "{-1;0;1}"
        annotation (Placement(transformation(extent={{-120,-62},{-80,-22}})));
      Modelica.Blocks.Interfaces.RealInput kappa_set "[0;1]"
        annotation (Placement(transformation(extent={{-120,-140},{-80,-100}})));
      Modelica.Blocks.Interfaces.RealInput u_set_prim "[0;1]"
        annotation (Placement(transformation(extent={{-120,-100},{-80,-60}})));
      Modelica.Blocks.Interfaces.RealOutput m_dot_prod "kg/s"
        annotation (Placement(transformation(extent={{90,70},{110,90}})));
      Modelica.Blocks.Interfaces.RealOutput m_dot_cons "kg/s"
        annotation (Placement(transformation(extent={{90,30},{110,50}})));
      Modelica.Blocks.Interfaces.RealOutput pump_contr "[0;1]"
        annotation (Placement(transformation(extent={{90,-50},{110,-30}})));
      Modelica.Blocks.Interfaces.RealOutput valve_contr "[0;1]"
        annotation (Placement(transformation(extent={{90,-90},{110,-70}})));
      Modelica.Blocks.Interfaces.RealInput m_dot_prim_is "kg/s"
        annotation (Placement(transformation(extent={{120,-140},{80,-100}})));
      Modelica.Blocks.Interfaces.RealOutput V_dot_prim_is annotation (Placement(
            transformation(
            extent={{-10,-10},{10,10}},
            rotation=-90,
            origin={0,-160})));
      Modelica.Blocks.Math.Product mass2volume_flow1
        annotation (Placement(transformation(extent={{48,-132},{28,-112}})));
      Modelica.Blocks.Sources.Constant factor3(k=(60000)*(1/Medium.d_const))
        annotation (Placement(transformation(extent={{82,-108},{62,-88}})));
      Modelica.Blocks.Math.RealToInteger realToInteger
        annotation (Placement(transformation(extent={{-72,-10},{-52,10}})));
      Modelica.Blocks.Math.RealToInteger realToInteger1
        annotation (Placement(transformation(extent={{-72,-52},{-52,-32}})));
      Controls.PrimaryFlowControl priFlowCon1
        annotation (Placement(transformation(extent={{-32,246},{-12,226}})));
      Modelica.Blocks.Interfaces.RealInput u_set_sec_cons "[0;1]" annotation (
          Placement(transformation(extent={{-116,138},{-76,178}}),
            iconTransformation(extent={{-116,138},{-76,178}})));
      Modelica.Blocks.Interfaces.RealInput u_set_sec_prod "[0;1]" annotation (
          Placement(transformation(extent={{-116,98},{-76,138}}),
            iconTransformation(extent={{-116,98},{-76,138}})));
      Modelica.Blocks.Interfaces.RealOutput pump_y1
        "0-1 voltage for production pump"
        annotation (Placement(transformation(extent={{96,170},{116,190}}),
            iconTransformation(extent={{96,170},{116,190}})));
      Modelica.Blocks.Interfaces.RealOutput pump_y2
        "0-1 voltage for consumption pump"
        annotation (Placement(transformation(extent={{98,146},{118,166}}),
            iconTransformation(extent={{98,146},{118,166}})));
      Modelica.Blocks.Math.Product mass2volume_flow
        annotation (Placement(transformation(extent={{26,76},{6,96}})));
      Modelica.Blocks.Sources.Constant factor2(k=(60000)*(1/Medium.d_const))
        annotation (Placement(transformation(extent={{60,64},{40,84}})));
      Modelica.Blocks.Interfaces.RealInput m_dot_sec_is "kg/s"
        annotation (Placement(transformation(extent={{120,108},{80,148}})));
      Modelica.Blocks.Interfaces.RealOutput V_dot_sec_is annotation (Placement(
            transformation(
            extent={{-10,-10},{10,10}},
            rotation=90,
            origin={50,244})));
    equation
      connect(factor1.y, volume2mass_flow.u1) annotation (Line(points={{-31,50},{-18,
              50},{-18,38},{-12,38}}, color={0,0,127}));
      connect(volume2mass_flow.y, secFlowCon.m_flow_set)
        annotation (Line(points={{11,32},{40,32},{40,26}}, color={0,0,127}));
      connect(secFlowCon.m_flow_production, m_dot_prod) annotation (Line(points={{51,19.1},
              {51,32},{84,32},{84,64},{74,64},{74,80},{100,80}},       color={0,0,127}));
      connect(secFlowCon.m_flow_consumption, m_dot_cons) annotation (Line(points={{51,8.9},
              {62,8.9},{62,28},{82,28},{82,40},{100,40}},      color={0,0,127}));
      connect(priFlowCon.pump_y, pump_contr) annotation (Line(points={{13,-30.9},{84,
              -30.9},{84,-40},{100,-40}}, color={0,0,127}));
      connect(priFlowCon.valve_op, valve_contr) annotation (Line(points={{13,-41.1},
              {82,-41.1},{82,-80},{100,-80}}, color={0,0,127}));
      connect(lin.op, priFlowCon.valve_op_set)
        annotation (Line(points={{-19,-70},{6,-70},{6,-48}}, color={0,0,127}));
      connect(kappa_set, lin.kappa) annotation (Line(points={{-100,-120},{-52,-120},
              {-52,-70},{-42,-70}}, color={0,0,127}));
      connect(u_set_prim, priFlowCon.pump_y_set) annotation (Line(points={{-100,
              -80},{-54,-80},{-54,-54},{-10,-54},{-10,-56},{-2,-56},{-2,-48}},
            color={0,0,127}));
      connect(factor3.y, mass2volume_flow1.u1) annotation (Line(points={{61,-98},
              {60,-98},{60,-116},{50,-116}},
                                         color={0,0,127}));
      connect(m_dot_prim_is, mass2volume_flow1.u2) annotation (Line(points={{100,
              -120},{58,-120},{58,-128},{50,-128}},
                                              color={0,0,127}));
      connect(mass2volume_flow1.y, V_dot_prim_is)
        annotation (Line(points={{27,-122},{0,-122},{0,-160}}, color={0,0,127}));
      connect(realToInteger.y, secFlowCon.pi)
        annotation (Line(points={{-51,0},{20,0},{20,8},{28,8}}, color={255,127,0}));
      connect(realToInteger.y, priFlowCon.pi)
        annotation (Line(points={{-51,0},{-28,0},{-28,-30},{-10,-30}}, color={255,127,0}));
      connect(realToInteger.u, pi)
        annotation (Line(points={{-74,0},{-100,0}}, color={0,0,127}));
      connect(mu, realToInteger1.u)
        annotation (Line(points={{-100,-42},{-74,-42}}, color={0,0,127}));
      connect(realToInteger1.y, priFlowCon.mu)
        annotation (Line(points={{-51,-42},{-10,-42}}, color={255,127,0}));
      connect(realToInteger1.y, secFlowCon.mu) annotation (Line(points={{-51,-42},{-32,-42},
              {-32,14},{2,14},{2,20},{28,20}}, color={255,127,0}));
      connect(realToInteger.y, priFlowCon1.pi) annotation (Line(points={{-51,0},
              {-26,0},{-26,150},{-44,150},{-44,242},{-34,242}}, color={255,127,
              0}));
      connect(priFlowCon.mu, priFlowCon1.mu) annotation (Line(points={{-10,-42},
              {-36,-42},{-36,0},{-24,0},{-24,216},{-40,216},{-40,222},{-42,222},
              {-42,230},{-34,230}}, color={255,127,0}));
      connect(u_set_sec_cons, priFlowCon1.pump_y_set) annotation (Line(points={
              {-96,158},{-42,158},{-42,214},{-26,214},{-26,224}}, color={0,0,
              127}));
      connect(u_set_sec_prod, priFlowCon1.valve_op_set) annotation (Line(points
            ={{-96,118},{-18,118},{-18,224}}, color={0,0,127}));
      connect(priFlowCon1.pump_y, pump_y1) annotation (Line(points={{-11,241.1},
              {12.5,241.1},{12.5,180},{106,180}},color={0,0,127}));
      connect(priFlowCon1.valve_op, pump_y2) annotation (Line(points={{-11,
              230.9},{18,230.9},{18,156},{108,156}},color={0,0,127}));
      connect(u_set_sec_prod, volume2mass_flow.u2) annotation (Line(points={{-96,
              118},{-22,118},{-22,26},{-12,26}}, color={0,0,127}));
      connect(factor2.y,mass2volume_flow. u2) annotation (Line(points={{39,74},
              {28,74},{28,80}}, color={0,0,127}));
      connect(m_dot_sec_is,mass2volume_flow. u1) annotation (Line(points={{100,128},
              {38,128},{38,92},{28,92}}, color={0,0,127}));
      connect(mass2volume_flow.y, V_dot_sec_is) annotation (Line(points={{5,86},
              {0,86},{0,260},{50,260},{50,244}}, color={0,0,127}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                -160},{100,200}}),
                             graphics={Rectangle(
              extent={{-100,180},{100,-160}},
              lineColor={28,108,200},
              fillColor={171,171,171},
              fillPattern=FillPattern.Solid), Text(
              extent={{-137,35},{137,-35}},
              textColor={0,0,0},
              origin={-1,1},
              rotation=90,
              textString="Conversion",
              textStyle={TextStyle.Bold})}),
                              Diagram(coordinateSystem(preserveAspectRatio=false,
              extent={{-100,-160},{100,200}})));
    end Conversion;

    package Validation
      package Validation

        package validation
          model Test_heat_transfer_station_production
            ProsNet.BidirectionalSubstation.New_Substation.HTS B1(
              n=0.5,
              redeclare ProsNet.Fluid.Pumps.Data.Pumps.IMP.NMTSmart25_120to180
                feedinPer,
              R_ins_transferpipe=100000) annotation (Placement(transformation(
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
            ProsNet.BidirectionalSubstation.Controller_PID_based.auxiliary.TimeTable_noInterp
              power_set1(table=[0,4; 3600,-4; 7200,10; 10800,-10; 14400,-6; 18000,6;
                  21600,10; 25200,10]) annotation (Placement(transformation(
                  extent={{-10,-10},{10,10}},
                  rotation=-90,
                  origin={-36,76})));
            ProsNet.BidirectionalSubstation.Controller_PID_based.auxiliary.TimeTable_noInterp
              temp_sec_in1(table=[0,55; 3600,30; 7200,55; 10800,30; 14400,30; 18000,
                  55; 21600,55; 25200,55]) annotation (Placement(transformation(
                  extent={{-10,-10},{10,10}},
                  rotation=-90,
                  origin={6,76})));
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
                                      bou(nPorts=1, redeclare final package
                Medium =
                  ProsNet.Media.Water)
              annotation (Placement(transformation(extent={{134,-152},{114,-132}})));
            Controller_PID_based.PID_Q_T_weighted_crossover_new
              pID_Q_T_weighted_crossover_new annotation (Placement(
                  transformation(extent={{-30,-18},{-6,16}})));
            Prosumers.SF1 sF11111111_1 annotation (Placement(transformation(
                    extent={{-28,-52},{-10,-40}})));
          equation
            connect(add.u1,realExpression. y) annotation (Line(points={{2,47},{2,49.5},
                    {22,49.5}},                color={0,0,127}));
            connect(temp_sec_in1.y,add. u2) annotation (Line(points={{6,65},{6,47},{
                    -4,47}},         color={0,0,127}));
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
            connect(power_set1.y, pID_Q_T_weighted_crossover_new.Q_dot_set)
              annotation (Line(points={{-36,65},{-36,26},{-24,26},{-24,16.8}},
                  color={0,0,127}));
            connect(add.y, pID_Q_T_weighted_crossover_new.T_sec_in_is)
              annotation (Line(points={{-1,35.5},{-2,35.5},{-2,26},{-12,26},{
                    -12,17}}, color={0,0,127}));
            connect(B1.contr_vars_real, pID_Q_T_weighted_crossover_new.contr_vars_real)
              annotation (Line(points={{2.2,-54},{10,-54},{10,-2},{-6,-2}},
                  color={0,0,127}));
            connect(B1.states, pID_Q_T_weighted_crossover_new.states)
              annotation (Line(points={{-38,-54},{-46,-54},{-46,-2},{-30,-2}},
                  color={0,0,127}));
            connect(sF11111111_1.port_b1, B1.port_b1) annotation (Line(points={{-16.6,
                    -37.1355},{-12.3333,-37.1355},{-12.3333,-37},{-9,-37}},
                  color={0,127,255}));
            connect(B1.port_a1, sF11111111_1.port_a1) annotation (Line(points={{-27.2,
                    -37},{-21.6952,-37},{-21.6952,-37.0194},{-17.5429,-37.0194}},
                                  color={0,127,255}));
            annotation (Diagram(graphics={                               Rectangle(extent={{-62,92},
                        {28,-82}},      lineColor={28,108,200})}));
          end Test_heat_transfer_station_production;

          model Test_heat_transfer_station_Consumption
            ProsNet.BidirectionalSubstation.New_Substation.HTS B2(
              n=0.5,
              redeclare ProsNet.Fluid.Pumps.Data.Pumps.IMP.NMTSmart25_120to180
                feedinPer,
              R_ins_transferpipe=100000) annotation (Placement(transformation(
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
            ProsNet.BidirectionalSubstation.Controller_PID_based.auxiliary.TimeTable_noInterp
              power_set2(table=[0,-10; 3600,10; 7200,-4; 10800,4; 14400,10; 18000,-10;
                  21600,-4; 25200,-4]) annotation (Placement(transformation(
                  extent={{-10,-10},{10,10}},
                  rotation=-90,
                  origin={-26,76})));
            ProsNet.BidirectionalSubstation.Controller_PID_based.auxiliary.TimeTable_noInterp
              temp_sec_in2(table=[0,30; 3600,55; 7200,30; 10800,55; 14400,55; 18000,
                  30; 21600,30; 25200,30]) annotation (Placement(transformation(
                  extent={{-10,-10},{10,10}},
                  rotation=-90,
                  origin={16,76})));
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
            Controller_PID_based.PID_Q_T_weighted_crossover_new
              pID_Q_T_weighted_crossover_new(alpha_sec_cons=1) annotation (
                Placement(transformation(extent={{-10,-16},{14,18}})));
            Prosumers.SF3 sF3_1 annotation (Placement(transformation(extent={{
                      -122,-86},{-38,-26}})));
          equation
            connect(realExpression1.y,add1. u1) annotation (Line(points={{24,47.5},{
                    24,45},{4,45}},             color={0,0,127}));
            connect(temp_sec_in2.y,add1. u2)
              annotation (Line(points={{16,65},{16,64},{-2,64},{-2,45}},
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
            connect(pID_Q_T_weighted_crossover_new.Q_dot_set, power_set2.y)
              annotation (Line(points={{-4,18.8},{-4,30},{-26,30},{-26,65}},
                  color={0,0,127}));
            connect(pID_Q_T_weighted_crossover_new.T_sec_in_is, add1.y)
              annotation (Line(points={{8,19},{1,19},{1,33.5}}, color={0,0,127}));
            connect(pID_Q_T_weighted_crossover_new.states, B2.states)
              annotation (Line(points={{-10,0},{-38,0},{-38,-54},{-28,-54}},
                  color={0,0,127}));
            connect(pID_Q_T_weighted_crossover_new.contr_vars_real, B2.contr_vars_real)
              annotation (Line(points={{14,0},{22,0},{22,-54},{12.2,-54}},
                  color={0,0,127}));
            connect(B2.port_b1, sF3_1.port_b1) annotation (Line(points={{1,-37},
                    {1,-24},{-40,-24},{-40,-16.8},{-66.4,-16.8}}, color={0,127,
                    255}));
            connect(B2.port_a1, sF3_1.port_a1) annotation (Line(points={{-17.2,
                    -37},{-17.2,-20},{-69,-20},{-69,-16.8}}, color={0,127,255}));
            annotation (Diagram(graphics={                               Rectangle(extent={{-52,92},
                        {38,-82}},      lineColor={28,108,200})}));
          end Test_heat_transfer_station_Consumption;
        end validation;

        package Tests "Testing the new models and especially controllers"

          model Test_heat_exchanger

            replaceable package Medium1 = ProsNet.Media.Water;
            replaceable package Medium2 = ProsNet.Media.Water;

            extends ProsNet.Prosumers.BaseClasses.PrimarySideParameters;
            extends
              ProsNet.Prosumers.SecondarySides.BaseClasses.PumpsPairDynParam;
            extends
              ProsNet.Prosumers.SecondarySides.BaseClasses.ControlVolumeDynParam;

            inner Modelica.Fluid.System system
              annotation (Placement(transformation(extent={{-30,54},{-10,74}})));
            Modelica.Fluid.Sources.MassFlowSource_T boundary(
              redeclare package Medium = Media.Water,
              m_flow=0.1,
              T=333.15,
              nPorts=1) annotation (Placement(transformation(extent={{-84,14},{-64,34}})));
            Modelica.Fluid.Sources.FixedBoundary boundary1(redeclare package
                Medium =
                  Media.Water, nPorts=1)
              annotation (Placement(transformation(extent={{-88,-90},{-68,-70}})));
            Modelica.Fluid.Sources.MassFlowSource_T boundary2(
              redeclare package Medium = Media.Water,
              use_m_flow_in=true,
              m_flow=1,
              T=313.15,
              nPorts=1) annotation (Placement(transformation(extent={{70,-34},{50,-14}})));
            Modelica.Fluid.Sources.FixedBoundary boundary3(redeclare package
                Medium =
                  Media.Water, nPorts=1)
              annotation (Placement(transformation(extent={{52,44},{32,64}})));
            Modelica.Blocks.Sources.Ramp ramp(
              height=0.1,
              duration=900,
              offset=0.1,
              startTime=900)
              annotation (Placement(transformation(extent={{50,-70},{70,-50}})));

            Fluid.HeatExchangers.LiquidToLiquid liquidToLiquid(
              redeclare package Medium1 = Medium1,
              redeclare package Medium2 = Medium2,
              m1_flow_nominal=1,
              m2_flow_nominal=1,
              dp1_nominal(displayUnit="bar") = 100000,
              dp2_nominal(displayUnit="bar") = 100000,
              use_Q_flow_nominal=true,
              Q_flow_nominal(displayUnit="kW") = 30000,
              T_a1_nominal=313.15,
              T_a2_nominal=333.15,
              eps_nominal=1)
              annotation (Placement(transformation(extent={{-24,-18},{-4,2}})));
            Fluid.Sensors.Temperature senTem2(redeclare package Medium = Medium1)
              annotation (Placement(transformation(extent={{46,12},{66,32}})));
            Fluid.Sensors.Temperature senTem1(redeclare package Medium = Medium2)
              annotation (Placement(transformation(extent={{-46,-70},{-26,-50}})));

            Real DeltaT1;
            Real DeltaT2;
          equation
            connect(ramp.y, boundary2.m_flow_in) annotation (Line(points={{71,-60},{76,-60},{76,
                    -16},{70,-16}},   color={0,0,127}));

            DeltaT2 =senTem2.T - liquidToLiquid.T_in2;
            DeltaT1 = senTem1.T - liquidToLiquid.T_in1;

            connect(boundary.ports[1], liquidToLiquid.port_a1)
              annotation (Line(points={{-64,24},{-62,24},{-62,-2},{-24,-2}}, color={0,127,255}));
            connect(boundary1.ports[1], liquidToLiquid.port_b2) annotation (Line(points={{-68,-80},
                    {-50,-80},{-50,-54},{-48,-54},{-48,-14},{-24,-14}}, color={0,127,255}));
            connect(liquidToLiquid.port_a2, boundary2.ports[1]) annotation (Line(points={{-4,-14},
                    {2,-14},{2,-18},{0,-18},{0,-24},{50,-24}}, color={0,127,255}));
            connect(liquidToLiquid.port_b1, boundary3.ports[1])
              annotation (Line(points={{-4,-2},{26,-2},{26,54},{32,54}}, color={0,127,255}));
            connect(senTem1.port, liquidToLiquid.port_b2) annotation (Line(points={{-36,-70},{-36,
                    -74},{-50,-74},{-50,-54},{-48,-54},{-48,-14},{-24,-14}}, color={0,127,255}));
            connect(senTem2.port, liquidToLiquid.port_b1)
              annotation (Line(points={{56,12},{56,-2},{-4,-2}}, color={0,127,255}));
            annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
                  coordinateSystem(preserveAspectRatio=false), graphics={Line(
                    points={{10,-34},{10,12}},
                    color={0,255,0},
                    thickness=1), Line(
                    points={{10,10},{14,4}},
                    color={0,255,0},
                    thickness=1)}),
              experiment(
                StopTime=2100,
                Interval=1,
                __Dymola_Algorithm="Dassl"));
          end Test_heat_exchanger;

          model Test_pipe_model
            Modelica.Fluid.Sensors.VolumeFlowRate volumeFlowRate(redeclare
                package Medium =
                  ProsNet.Media.Water)
              annotation (Placement(transformation(extent={{-20,32},{0,52}})));
            Modelica.Fluid.Sensors.RelativePressure relativePressure(redeclare
                package Medium =
                         ProsNet.Media.Water)
              annotation (Placement(transformation(extent={{20,64},{40,84}})));
            Modelica.Fluid.Sensors.RelativeTemperature relativeTemperature(redeclare
                package Medium = ProsNet.Media.Water)
              annotation (Placement(transformation(extent={{20,4},{40,-16}})));
            Modelica.Fluid.Sources.MassFlowSource_T mass_source(
              redeclare package Medium = ProsNet.Media.Water,
              use_m_flow_in=true,
              use_T_in=true,
              m_flow=1,
              T(displayUnit="K"),
              nPorts=1) annotation (Placement(transformation(extent={{-94,32},{-74,52}})));
            inner Modelica.Fluid.System system(
              T_ambient=285.15,
              allowFlowReversal=true,
              energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyStateInitial)
              annotation (Placement(transformation(extent={{132,76},{152,96}})));
            Modelica.Fluid.Sources.Boundary_pT bou(redeclare package Medium =
                  ProsNet.Media.Water, nPorts=1)
              annotation (Placement(transformation(extent={{104,32},{84,52}})));
            ProsNet.BidirectionalSubstation.Controller_PID_based.auxiliary.TimeTable_noInterp
              volume_flow(table=[0,1; 10,5; 20,10; 30,11.4; 40,15; 50,20; 60,1; 70,5;
                  80,10; 90,11.4; 100,15; 110,20; 120,1; 130,5; 140,10; 150,11.4; 160,
                  15; 170,20; 7470,20; 7480,0; 10980,20; 17580,20], timeScale=1)
              annotation (Placement(transformation(extent={{-182,50},{-162,70}})));
            ProsNet.BidirectionalSubstation.Controller_PID_based.auxiliary.TimeTable_noInterp
              temperatures(
              table=[0,30; 10,30; 20,30; 30,30; 40,30; 50,30; 60,60; 70,60; 80,60; 90,
                  60; 100,60; 110,60; 120,90; 130,90; 140,90; 150,90; 160,90; 170,90;
                  180,90; 7480,90; 10980,90; 17580,90],
              timeScale=1,
              y(unit="K"))
              annotation (Placement(transformation(extent={{-182,16},{-162,36}})));
            Modelica.Blocks.Math.Gain gain(k=1/60.266)
              annotation (Placement(transformation(extent={{-138,50},{-118,70}})));
            Modelica.Blocks.Math.UnitConversions.From_degC from_degC
              annotation (Placement(transformation(extent={{-138,16},{-118,36}})));
            ProsNet.Fluid.Sensors.TemperatureTwoPort Tem_in(
              redeclare package Medium = ProsNet.Media.Water,
              m_flow_nominal=1/6,
              tau=0) annotation (Placement(transformation(extent={{-54,32},{-34,52}})));
            ProsNet.Fluid.Sensors.TemperatureTwoPort Tem_out(
              redeclare package Medium = ProsNet.Media.Water,
              m_flow_nominal=1/6,
              tau=0) annotation (Placement(transformation(extent={{54,32},{74,52}})));

            Real DeltaT;
            Modelica.Fluid.Sensors.VolumeFlowRate volumeFlowRate1(redeclare
                package Medium =
                         ProsNet.Media.Water)
              annotation (Placement(transformation(extent={{-18,-80},{2,-60}})));
            Modelica.Fluid.Sensors.RelativePressure relativePressure1(redeclare
                package Medium =
                         ProsNet.Media.Water)
              annotation (Placement(transformation(extent={{22,-48},{42,-28}})));
            Modelica.Fluid.Sensors.RelativeTemperature relativeTemperature1(redeclare
                package Medium = ProsNet.Media.Water)
              annotation (Placement(transformation(extent={{22,-108},{42,-128}})));
            Modelica.Fluid.Sources.Boundary_pT bou1(redeclare package Medium =
                  ProsNet.Media.Water, nPorts=1)
              annotation (Placement(transformation(extent={{106,-80},{86,-60}})));
            ProsNet.Fluid.Sensors.TemperatureTwoPort Tem_in1(
              redeclare package Medium = ProsNet.Media.Water,
              m_flow_nominal=1/6,
              tau=0) annotation (Placement(transformation(extent={{-52,-80},{-32,-60}})));
            ProsNet.Fluid.Sensors.TemperatureTwoPort Tem_out1(
              redeclare package Medium = ProsNet.Media.Water,
              m_flow_nominal=1/6,
              tau=0) annotation (Placement(transformation(extent={{56,-80},{76,-60}})));
            ProsNet.Fluid.Pipes.InsulatedPipe_plug pipe_new(
              allowFlowReversal=true,
              length=1000,
              energyDynamics=Modelica.Fluid.Types.Dynamics.DynamicFreeInitial)
              annotation (Placement(transformation(extent={{18,-80},{38,-60}})));
            Modelica.Fluid.Sources.MassFlowSource_T mass_source1(
              redeclare package Medium = ProsNet.Media.Water,
              use_m_flow_in=true,
              use_T_in=true,
              m_flow=1,
              T(displayUnit="K"),
              nPorts=1) annotation (Placement(transformation(extent={{-88,-80},{-68,-60}})));
            ProsNet.Fluid.Pipes.InsulatedPipe pipe(
              allowFlowReversal=true,
              length=1000,
              energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyStateInitial)
              annotation (Placement(transformation(extent={{18,30},{38,50}})));
          equation
            connect(volume_flow.y, gain.u) annotation (Line(
                points={{-161,60},{-161,60},{-142,60},{-140,60}},
                color={0,0,127},
                smooth=Smooth.Bezier));
            connect(temperatures.y, from_degC.u) annotation (Line(
                points={{-161,26},{-161,26},{-144,26},{-140,26}},
                color={0,0,127},
                smooth=Smooth.Bezier));
            connect(mass_source.ports[1], Tem_in.port_a) annotation (Line(
                points={{-74,42},{-54,42}},
                color={0,127,255},
                smooth=Smooth.Bezier));
            connect(Tem_in.port_b, volumeFlowRate.port_a) annotation (Line(
                points={{-34,42},{-20,42}},
                color={0,127,255},
                smooth=Smooth.Bezier));
            connect(Tem_out.port_b, bou.ports[1]) annotation (Line(
                points={{74,42},{84,42}},
                color={0,127,255},
                smooth=Smooth.Bezier));

            DeltaT = Tem_out.T - Tem_in.T;
            connect(volumeFlowRate.port_b, relativePressure.port_a) annotation (Line(
                  points={{0,42},{8,42},{8,74},{20,74}}, color={0,127,255}));
            connect(relativePressure.port_b, Tem_out.port_a) annotation (Line(points={{40,
                    74},{48,74},{48,42},{54,42}}, color={0,127,255}));
            connect(relativeTemperature.port_b, Tem_out.port_a) annotation (Line(points={
                    {40,-6},{46,-6},{46,42},{54,42}}, color={0,127,255}));
            connect(relativeTemperature.port_a, volumeFlowRate.port_b) annotation (Line(
                  points={{20,-6},{10,-6},{10,42},{0,42}}, color={0,127,255}));
            connect(Tem_in1.port_b, volumeFlowRate1.port_a) annotation (Line(
                points={{-32,-70},{-18,-70}},
                color={0,127,255},
                smooth=Smooth.Bezier));
            connect(Tem_out1.port_b, bou1.ports[1]) annotation (Line(
                points={{76,-70},{86,-70}},
                color={0,127,255},
                smooth=Smooth.Bezier));
            connect(volumeFlowRate1.port_b, relativePressure1.port_a) annotation (Line(
                  points={{2,-70},{10,-70},{10,-38},{22,-38}}, color={0,127,255}));
            connect(relativePressure1.port_b, Tem_out1.port_a) annotation (Line(points={{
                    42,-38},{50,-38},{50,-70},{56,-70}}, color={0,127,255}));
            connect(relativeTemperature1.port_b, Tem_out1.port_a) annotation (Line(points=
                   {{42,-118},{50,-118},{50,-70},{56,-70}}, color={0,127,255}));
            connect(relativeTemperature1.port_a, volumeFlowRate1.port_b) annotation (Line(
                  points={{22,-118},{10,-118},{10,-70},{2,-70}}, color={0,127,255}));
            connect(volumeFlowRate1.port_b, pipe_new.port_a)
              annotation (Line(points={{2,-70},{18,-70}}, color={0,127,255}));
            connect(pipe_new.port_b, Tem_out1.port_a)
              annotation (Line(points={{38,-70},{56,-70}}, color={0,127,255}));
            connect(mass_source1.ports[1], Tem_in1.port_a)
              annotation (Line(points={{-68,-70},{-52,-70}}, color={0,127,255}));
            connect(gain.y, mass_source.m_flow_in)
              annotation (Line(points={{-117,60},{-94,60},{-94,50}}, color={0,0,127}));
            connect(gain.y, mass_source1.m_flow_in) annotation (Line(points={{-117,60},{
                    -104,60},{-104,-56},{-88,-56},{-88,-62}}, color={0,0,127}));
            connect(from_degC.y, mass_source.T_in) annotation (Line(points={{-117,26},{
                    -102,26},{-102,46},{-96,46}}, color={0,0,127}));
            connect(from_degC.y, mass_source1.T_in) annotation (Line(points={{-117,26},{
                    -106,26},{-106,-66},{-90,-66}}, color={0,0,127}));
            connect(pipe.port_a, relativePressure.port_a) annotation (Line(points={{18,40},
                    {12,40},{12,42},{8,42},{8,74},{20,74}}, color={0,127,255}));
            connect(pipe.port_b, Tem_out.port_a) annotation (Line(points={{38,40},{42,40},
                    {42,42},{54,42}}, color={0,127,255}));
            annotation (                                                       experiment(
                StopTime=17500,
                Interval=0.1,
                __Dymola_Algorithm="Dassl"));
          end Test_pipe_model;

          model Test_pump_curve

            ProsNet.Fluid.Pumps.SpeedControlled_y
                                          pump_prim_prod(
              redeclare final package Medium = ProsNet.Media.Water,
              final energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyStateInitial,
              inputType=ProsNet.Fluid.Types.InputType.Continuous,
              final tau=1,
              redeclare final ProsNet.Fluid.Pumps.Data.Pumps.IMP.NMTPlus152025to40 per,
              final use_inputFilter=true,
              final riseTime=1,
              final init=Modelica.Blocks.Types.Init.SteadyState,
              final y_start=0)                annotation (Placement(transformation(
                  extent={{10,10},{-10,-10}},
                  rotation=-90,
                  origin={-44,-8})));
            ProsNet.Fluid.Sources.Boundary_pT
                                      bou(redeclare package Medium =
                  ProsNet.Media.Water,
                nPorts=1)
              annotation (Placement(transformation(extent={{68,36},{48,56}})));
            Modelica.Blocks.Sources.RealExpression realExpression(y=1)
              annotation (Placement(transformation(extent={{-168,2},{-148,22}})));
            Modelica.Blocks.Sources.Ramp ramp(
              height=-1,
              duration=1200,
              offset=1,
              startTime=5)
              annotation (Placement(transformation(extent={{140,-42},{120,-22}})));
            Modelica.Fluid.Valves.ValveLinear valveLinear(
              redeclare package Medium = ProsNet.Media.Water,
              dp_start=0,
              dp_nominal=70000,
              m_flow_nominal=0.55)
              annotation (Placement(transformation(extent={{42,-20},{62,0}})));
          equation
            connect(realExpression.y, pump_prim_prod.y) annotation (Line(points={{-147,12},
                    {-64,12},{-64,-8},{-56,-8}}, color={0,0,127}));
            connect(pump_prim_prod.port_b, bou.ports[1]) annotation (Line(points={{-44,2},
                    {-44,20},{42,20},{42,46},{48,46}}, color={0,127,255}));
            connect(pump_prim_prod.port_b, valveLinear.port_a) annotation (Line(points={{-44,2},
                    {-44,20},{42,20},{42,2},{32,2},{32,-10},{42,-10}},        color={0,
                    127,255}));
            connect(valveLinear.port_b, pump_prim_prod.port_a) annotation (Line(points={{62,-10},
                    {64,-10},{64,-26},{-44,-26},{-44,-18}},         color={0,127,255}));
            connect(ramp.y, valveLinear.opening) annotation (Line(points={{119,-32},{66,
                    -32},{66,2},{52,2},{52,-2}}, color={0,0,127}));
            annotation ();
          end Test_pump_curve;

          model Test_check_valve_model
            Modelica.Fluid.Sensors.VolumeFlowRate volumeFlowRate(redeclare
                package Medium =
                  ProsNet.Media.Water)
              annotation (Placement(transformation(extent={{-20,32},{0,52}})));
            Modelica.Fluid.Sensors.RelativePressure relativePressure(redeclare
                package Medium =
                         ProsNet.Media.Water)
              annotation (Placement(transformation(extent={{20,64},{40,84}})));
            Modelica.Fluid.Sources.MassFlowSource_T mass_source(
              redeclare package Medium = ProsNet.Media.Water,
              use_m_flow_in=true,
              use_T_in=true,
              m_flow=1,
              T(displayUnit="K"),
              nPorts=1) annotation (Placement(transformation(extent={{-94,32},{-74,52}})));
            inner Modelica.Fluid.System system(
              T_ambient=285.15,
              allowFlowReversal=true,
              energyDynamics=Modelica.Fluid.Types.Dynamics.DynamicFreeInitial)
              annotation (Placement(transformation(extent={{64,-52},{84,-32}})));
            Modelica.Fluid.Sources.Boundary_pT bou(redeclare package Medium =
                  ProsNet.Media.Water,
              T=309.9,                 nPorts=1)
              annotation (Placement(transformation(extent={{104,32},{84,52}})));
            ProsNet.BidirectionalSubstation.Controller_PID_based.auxiliary.TimeTable_noInterp
              mass_flow_kg_s(table=[0,0; 900,0.1; 1800,0.2; 2700,0.3; 3600,0.4; 4500,
                  0.5; 5400,0.6; 6300,0.7; 7200,0.8; 8100,0.9; 9000,1; 9900,1.1;
                  10800,1.2; 11700,1.3; 12600,1.4; 13500,1.5; 14400,1.6; 15300,1.7;
                  16200,1.8; 17100,1.9; 18000,2], timeScale=1)
              annotation (Placement(transformation(extent={{-182,50},{-162,70}})));
            ProsNet.BidirectionalSubstation.Controller_PID_based.auxiliary.TimeTable_noInterp
              temperatures(
              table=[0,36.75; 900,36.75; 1800,36.75; 2700,36.75; 3600,36.75; 4500,
                  36.75; 5400,36.75; 6300,36.75; 7200,36.75; 8100,36.75; 9000,36.75;
                  9900,36.75; 10800,36.75; 11700,36.75; 12600,36.75; 13500,36.75;
                  14400,36.75; 15300,36.75; 16200,36.75; 17100,36.75; 18000,36.75],
              timeScale=1,
              y(unit="K"))
              annotation (Placement(transformation(extent={{-182,16},{-162,36}})));
            Modelica.Blocks.Math.UnitConversions.From_degC from_degC
              annotation (Placement(transformation(extent={{-138,16},{-118,36}})));
            ProsNet.Fluid.Sensors.TemperatureTwoPort Tem_in(
              redeclare package Medium = ProsNet.Media.Water,
              m_flow_nominal=1/6,
              tau=1) annotation (Placement(transformation(extent={{-54,32},{-34,52}})));
            ProsNet.Fluid.Sensors.TemperatureTwoPort Tem_out(
              redeclare package Medium = ProsNet.Media.Water,
              m_flow_nominal=1/6,
              tau=1) annotation (Placement(transformation(extent={{54,32},{74,52}})));

            Real DeltaT;
            Modelica.Blocks.Sources.Constant const(k=1)
              annotation (Placement(transformation(extent={{-20,66},{0,86}})));
            ProsNet.Fluid.FixedResistances.CheckValve cheVal_prim_cons(
              m_flow_nominal=1.25,
              redeclare final package Medium = ProsNet.Media.Water,
              final CvData=ProsNet.Fluid.Types.CvTypes.Kv,
              final Kv=6.29,
              final l=0.002) annotation (Placement(transformation(
                  extent={{10,-10},{-10,10}},
                  rotation=180,
                  origin={32,42})));
          equation
            connect(temperatures.y, from_degC.u) annotation (Line(
                points={{-161,26},{-161,26},{-144,26},{-140,26}},
                color={0,0,127},
                smooth=Smooth.Bezier));
            connect(from_degC.y, mass_source.T_in) annotation (Line(
                points={{-117,26},{-102,26},{-102,46},{-96,46}},
                color={0,0,127},
                smooth=Smooth.Bezier));
            connect(mass_source.ports[1], Tem_in.port_a) annotation (Line(
                points={{-74,42},{-54,42}},
                color={0,127,255},
                smooth=Smooth.Bezier));
            connect(Tem_in.port_b, volumeFlowRate.port_a) annotation (Line(
                points={{-34,42},{-20,42}},
                color={0,127,255},
                smooth=Smooth.Bezier));
            connect(Tem_out.port_b, bou.ports[1]) annotation (Line(
                points={{74,42},{84,42}},
                color={0,127,255},
                smooth=Smooth.Bezier));

            DeltaT = Tem_out.T - Tem_in.T;
            connect(mass_flow_kg_s.y, mass_source.m_flow_in) annotation (Line(points=
                    {{-161,60},{-128,60},{-128,50},{-94,50}}, color={0,0,127}));
            connect(cheVal_prim_cons.port_b, Tem_out.port_a)
              annotation (Line(points={{42,42},{54,42}}, color={0,127,255}));
            connect(volumeFlowRate.port_b, cheVal_prim_cons.port_a)
              annotation (Line(points={{0,42},{22,42}}, color={0,127,255}));
            connect(relativePressure.port_a, cheVal_prim_cons.port_a)
              annotation (Line(points={{20,74},{22,74},{22,42}}, color={0,127,255}));
            connect(relativePressure.port_b, cheVal_prim_cons.port_b) annotation (
                Line(points={{40,74},{42,74},{42,42},{42,42}}, color={0,127,255}));
            annotation (                                                       experiment(
                StopTime=18000,
                Interval=10,
                __Dymola_Algorithm="Dassl"));
          end Test_check_valve_model;

          model Test_control_valve_model
            Modelica.Fluid.Sensors.VolumeFlowRate volumeFlowRate(redeclare
                package Medium =
                  ProsNet.Media.Water)
              annotation (Placement(transformation(extent={{-20,32},{0,52}})));
            Modelica.Fluid.Sensors.RelativePressure relativePressure(redeclare
                package Medium =
                         ProsNet.Media.Water)
              annotation (Placement(transformation(extent={{20,64},{40,84}})));
            Modelica.Fluid.Sources.MassFlowSource_T mass_source(
              redeclare package Medium = ProsNet.Media.Water,
              use_m_flow_in=true,
              use_T_in=true,
              m_flow=1,
              T(displayUnit="K"),
              nPorts=1) annotation (Placement(transformation(extent={{-94,32},{-74,52}})));
            inner Modelica.Fluid.System system(
              T_ambient=285.15,
              allowFlowReversal=true,
              energyDynamics=Modelica.Fluid.Types.Dynamics.DynamicFreeInitial)
              annotation (Placement(transformation(extent={{64,-52},{84,-32}})));
            Modelica.Fluid.Sources.Boundary_pT bou(redeclare package Medium =
                  ProsNet.Media.Water,
              T=309.9,                 nPorts=1)
              annotation (Placement(transformation(extent={{104,32},{84,52}})));
            ProsNet.BidirectionalSubstation.Controller_PID_based.auxiliary.TimeTable_noInterp
              mass_flow_kg_s(table=[0,0; 900,0.1; 1800,0.2; 2700,0.3; 3600,0.4; 4500,
                  0.5; 5400,0.6; 6300,0.7; 7200,0.8; 8100,0.9; 9000,1; 9900,1.1;
                  10800,1.2; 11700,1.3; 12600,1.4; 13500,1.5; 14400,1.6; 15300,1.7;
                  16200,1.8; 17100,1.9; 18000,2], timeScale=1)
              annotation (Placement(transformation(extent={{-182,50},{-162,70}})));
            ProsNet.BidirectionalSubstation.Controller_PID_based.auxiliary.TimeTable_noInterp
              temperatures(
              table=[0,36.75; 900,36.75; 1800,36.75; 2700,36.75; 3600,36.75; 4500,
                  36.75; 5400,36.75; 6300,36.75; 7200,36.75; 8100,36.75; 9000,36.75;
                  9900,36.75; 10800,36.75; 11700,36.75; 12600,36.75; 13500,36.75;
                  14400,36.75; 15300,36.75; 16200,36.75; 17100,36.75; 18000,36.75],
              timeScale=1,
              y(unit="K"))
              annotation (Placement(transformation(extent={{-182,16},{-162,36}})));
            Modelica.Blocks.Math.UnitConversions.From_degC from_degC
              annotation (Placement(transformation(extent={{-138,16},{-118,36}})));
            ProsNet.Fluid.Sensors.TemperatureTwoPort Tem_in(
              redeclare package Medium = ProsNet.Media.Water,
              m_flow_nominal=1/6,
              tau=1) annotation (Placement(transformation(extent={{-54,32},{-34,52}})));
            ProsNet.Fluid.Sensors.TemperatureTwoPort Tem_out(
              redeclare package Medium = ProsNet.Media.Water,
              m_flow_nominal=1/6,
              tau=1) annotation (Placement(transformation(extent={{54,32},{74,52}})));

            Real DeltaT;
            ProsNet.Fluid.Valves.TwoWayEqualPercentage valve_prim_cons(
              m_flow_nominal=1.25,
              kFixed=0,
              redeclare final package Medium = ProsNet.Media.Water,
              final CvData=ProsNet.Fluid.Types.CvTypes.Kv,
              final Kv=6.29,
              final use_inputFilter=true,
              final riseTime=35,
              final init=Modelica.Blocks.Types.Init.InitialState,
              final y_start=0,
              final l=0.002) annotation (Placement(transformation(
                  extent={{-10,10},{10,-10}},
                  rotation=180,
                  origin={26,34})));
            Modelica.Blocks.Sources.Constant const(k=1)
              annotation (Placement(transformation(extent={{-20,66},{0,86}})));
          equation
            connect(temperatures.y, from_degC.u) annotation (Line(
                points={{-161,26},{-161,26},{-144,26},{-140,26}},
                color={0,0,127},
                smooth=Smooth.Bezier));
            connect(from_degC.y, mass_source.T_in) annotation (Line(
                points={{-117,26},{-102,26},{-102,46},{-96,46}},
                color={0,0,127},
                smooth=Smooth.Bezier));
            connect(mass_source.ports[1], Tem_in.port_a) annotation (Line(
                points={{-74,42},{-54,42}},
                color={0,127,255},
                smooth=Smooth.Bezier));
            connect(Tem_in.port_b, volumeFlowRate.port_a) annotation (Line(
                points={{-34,42},{-20,42}},
                color={0,127,255},
                smooth=Smooth.Bezier));
            connect(Tem_out.port_b, bou.ports[1]) annotation (Line(
                points={{74,42},{84,42}},
                color={0,127,255},
                smooth=Smooth.Bezier));

            DeltaT = Tem_out.T - Tem_in.T;
            connect(mass_flow_kg_s.y, mass_source.m_flow_in) annotation (Line(points=
                    {{-161,60},{-128,60},{-128,50},{-94,50}}, color={0,0,127}));
            connect(const.y, valve_prim_cons.y) annotation (Line(points={{1,76},{14,
                    76},{14,46},{26,46}}, color={0,0,127}));
            connect(volumeFlowRate.port_b, valve_prim_cons.port_b) annotation (Line(
                  points={{0,42},{10,42},{10,34},{16,34}}, color={0,127,255}));
            connect(valve_prim_cons.port_a, Tem_out.port_a) annotation (Line(points={
                    {36,34},{46,34},{46,42},{54,42}}, color={0,127,255}));
            connect(relativePressure.port_b, valve_prim_cons.port_a)
              annotation (Line(points={{40,74},{40,34},{36,34}}, color={0,127,255}));
            connect(valve_prim_cons.port_b, relativePressure.port_a) annotation (Line(
                  points={{16,34},{6,34},{6,74},{20,74}}, color={0,127,255}));
            annotation (                                                       experiment(
                StopTime=18000,
                Interval=10,
                __Dymola_Algorithm="Dassl"));
          end Test_control_valve_model;

          model Test_TwoProsumers
            HTS B1(n=0.5, redeclare
                Fluid.Pumps.Data.Pumps.IMP.NMTSmart25_120to180 feedinPer)
              annotation (Placement(transformation(
                  extent={{20,-18},{-20,18}},
                  rotation=0,
                  origin={-48,8})));
            Controller_PID_based.PID_Q_T_weighted_sameside Ctrl1(alpha_prim_prod=0.35,
                alpha_sec_cons=0.35) annotation (Placement(transformation(
                  extent={{-12,-17},{12,17}},
                  rotation=0,
                  origin={-44,73})));
            Controller_PID_based.auxiliary.TimeTable_noInterp power_set1(table=[0,10; 900,10;
                  1800,10; 2700,10; 3600,-10; 4500,-4; 5400,4; 6300,4])  annotation (Placement(
                  transformation(
                  extent={{-10,-10},{10,10}},
                  rotation=-90,
                  origin={-70,134})));
            Controller_PID_based.auxiliary.TimeTable_noInterp temp_sec_in1(table=[0,55; 900,55;
                  1800,55; 2700,55; 3600,30; 4500,30; 5400,55; 6300,55])   annotation (
                Placement(transformation(
                  extent={{-10,-10},{10,10}},
                  rotation=-90,
                  origin={-28,134})));
            Fluid.Pipes.InsulatedPipe_plug pipe_hot12
              annotation (Placement(transformation(extent={{-8,-58},{18,-32}})));
            HTS B2(n=0.5, redeclare
                Fluid.Pumps.Data.Pumps.IMP.NMTSmart25_120to180 feedinPer)
              annotation (Placement(transformation(
                  extent={{20,-18},{-20,18}},
                  rotation=0,
                  origin={50,8})));
            Controller_PID_based.PID_Q_T_weighted_sameside Ctrl2(alpha_prim_prod=0.35,
                alpha_sec_cons=0.35) annotation (Placement(transformation(
                  extent={{-12,-17},{12,17}},
                  rotation=0,
                  origin={52,73})));
            Controller_PID_based.auxiliary.TimeTable_noInterp power_set2(table=[0,-10; 900,-10;
                  1800,-10; 2700,-10; 3600,10; 4500,4; 5400,-4; 6300,-4])
                                                                         annotation (Placement(
                  transformation(
                  extent={{-10,-10},{10,10}},
                  rotation=-90,
                  origin={28,134})));
            Controller_PID_based.auxiliary.TimeTable_noInterp temp_sec_in2(table=[0,30; 900,30;
                  1800,30; 2700,30; 3600,55; 4500,55; 5400,30; 6300,30])   annotation (
                Placement(transformation(
                  extent={{-10,-10},{10,10}},
                  rotation=-90,
                  origin={70,134})));
            Fluid.Pipes.InsulatedPipe_plug pipe_cold12
              annotation (Placement(transformation(extent={{18,-103},{-8,-77}})));
            Modelica.Fluid.Sources.Boundary_pT boundary(
              redeclare package Medium = Media.Water,
              use_p_in=false,
              T=325.4,
              nPorts=1) annotation (Placement(transformation(extent={{-92,-55},{-72,-35}})));
            inner Modelica.Fluid.System system(T_ambient=285.15)
              annotation (Placement(transformation(extent={{-92,-114},{-72,-94}})));
            Modelica.Blocks.Math.Add add annotation (Placement(transformation(
                  extent={{-5,-5},{5,5}},
                  rotation=-90,
                  origin={-31,103})));
            Modelica.Blocks.Sources.RealExpression realExpression(y=273.15) annotation (Placement(
                  transformation(
                  extent={{-5,-6},{5,6}},
                  rotation=270,
                  origin={-8,117})));
            Modelica.Blocks.Sources.RealExpression realExpression1(y=273.15) annotation (
                Placement(transformation(
                  extent={{-5,-6},{5,6}},
                  rotation=270,
                  origin={82,115})));
            Modelica.Blocks.Math.Add add1 annotation (Placement(transformation(
                  extent={{-5,-5},{5,5}},
                  rotation=-90,
                  origin={59,101})));
          equation
            connect(B1.contr_vars_real, Ctrl1.contr_vars_real)
              annotation (Line(points={{-27.8,8},{-20,8},{-20,72},{-32,72}}, color={0,0,127}));
            connect(Ctrl1.states, B1.states)
              annotation (Line(points={{-56,72},{-74,72},{-74,8},{-68,8}}, color={0,0,127}));
            connect(power_set1.y, Ctrl1.Q_dot_set) annotation (Line(points={{-70,123},{-70,118},{
                    -50,118},{-50,90.8}},  color={0,0,127}));
            connect(B2.contr_vars_real,Ctrl2. contr_vars_real)
              annotation (Line(points={{70.2,8},{78,8},{78,72},{64,72}},     color={0,0,127}));
            connect(Ctrl2.states,B2. states)
              annotation (Line(points={{40,72},{24,72},{24,8},{30,8}},     color={0,0,127}));
            connect(power_set2.y,Ctrl2. Q_dot_set) annotation (Line(points={{28,123},{28,118},{
                    46,118},{46,90.8}},    color={0,0,127}));
            connect(B1.hot_prim, pipe_hot12.port_a)
              annotation (Line(points={{-34,-10.2},{-34,-45},{-8,-45}}, color={0,127,255}));
            connect(pipe_hot12.port_b, B2.hot_prim)
              annotation (Line(points={{18,-45},{64,-45},{64,-10.2}}, color={0,127,255}));
            connect(B1.cold_prim, pipe_cold12.port_b)
              annotation (Line(points={{-62,-10},{-62,-90},{-8,-90}}, color={0,127,255}));
            connect(pipe_cold12.port_a, B2.cold_prim)
              annotation (Line(points={{18,-90},{36,-90},{36,-10}}, color={0,127,255}));
            connect(pipe_hot12.port_a, boundary.ports[1])
              annotation (Line(points={{-8,-45},{-72,-45}}, color={0,127,255}));
            connect(add.y, Ctrl1.T_sec_in_is)
              annotation (Line(points={{-31,97.5},{-31,91},{-38,91}}, color={0,0,127}));
            connect(temp_sec_in1.y, add.u2) annotation (Line(points={{-28,123},{-28,114},{-34,114},
                    {-34,109}}, color={0,0,127}));
            connect(add.u1, realExpression.y) annotation (Line(points={{-28,109},{-12,109},{-12,
                    106},{-8,106},{-8,111.5}}, color={0,0,127}));
            connect(temp_sec_in2.y, add1.u2)
              annotation (Line(points={{70,123},{70,114},{56,114},{56,107}}, color={0,0,127}));
            connect(add1.y, Ctrl2.T_sec_in_is)
              annotation (Line(points={{59,95.5},{58,95.5},{58,91}}, color={0,0,127}));
            connect(realExpression1.y, add1.u1) annotation (Line(points={{82,109.5},{82,104},{68,
                    104},{68,107},{62,107}}, color={0,0,127}));
            annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-120},{
                      200,160}})),                                         Diagram(
                  coordinateSystem(preserveAspectRatio=false, extent={{-100,-120},{200,160}}),
                                                               graphics={Rectangle(extent={{-92,
                        154},{-2,-20}}, lineColor={28,108,200}),         Rectangle(extent={{6,
                        154},{96,-20}}, lineColor={28,108,200})}),
              experiment(
                StopTime=6300,
                Interval=0.1,
                __Dymola_Algorithm="Dassl"));
          end Test_TwoProsumers;
        end Tests;
      end Validation;

      model Test_Conversion
        Conversion conversion
          annotation (Placement(transformation(extent={{-6,-2},{14,32}})));
        Modelica.Blocks.Sources.RealExpression T_sec_in(y=65)
          annotation (Placement(transformation(extent={{-86,38},{-66,58}})));
        Modelica.Blocks.Sources.IntegerExpression pi(y=0)
          annotation (Placement(transformation(extent={{-86,-2},{-66,18}})));
        Modelica.Blocks.Sources.RealExpression V_dot_sec_set(y=5)
          annotation (Placement(transformation(extent={{-86,18},{-66,38}})));
        Modelica.Blocks.Sources.IntegerExpression mu(y=-1)
          annotation (Placement(transformation(extent={{-86,-24},{-66,-4}})));
        Modelica.Blocks.Sources.RealExpression u_set(y=0.5)
          annotation (Placement(transformation(extent={{-86,-48},{-66,-28}})));
        Modelica.Blocks.Sources.RealExpression kappa_set(y=0.8)
          annotation (Placement(transformation(extent={{-86,-66},{-66,-46}})));
        Modelica.Blocks.Sources.RealExpression m_dot_sec_is(y=0.2)
          annotation (Placement(transformation(extent={{76,16},{56,36}})));
        Modelica.Blocks.Sources.RealExpression m_dot_prim_is(y=0.02)
          annotation (Placement(transformation(extent={{76,-10},{56,10}})));
      equation
        connect(kappa_set.y, conversion.kappa_set) annotation (Line(points={{-65,-56},
                {-12,-56},{-12,1.77778},{-6,1.77778}},
                                                color={0,0,127}));
        connect(u_set.y, conversion.u_set_prim) annotation (Line(points={{-65,-38},
                {-14,-38},{-14,5.55556},{-6,5.55556}},
                                           color={0,0,127}));
        connect(mu.y, conversion.mu) annotation (Line(points={{-65,-14},{-16,
                -14},{-16,9.14444},{-6,9.14444}},
                                    color={255,127,0}));
        connect(pi.y, conversion.pi) annotation (Line(points={{-65,8},{-18,8},{
                -18,13.1111},{-6,13.1111}},
                              color={255,127,0}));
        connect(V_dot_sec_set.y, conversion.V_dot_sec_set) annotation (Line(points=
                {{-65,28},{-14,28},{-14,24},{-6,24}}, color={0,0,127}));
        connect(T_sec_in.y, conversion.T_sec_in_set) annotation (Line(points={{-65,
                48},{-12,48},{-12,28},{-6,28}}, color={0,0,127}));
        connect(conversion.m_dot_sec_is, m_dot_sec_is.y)
          annotation (Line(points={{14,25.2},{34,25.2},{34,26},{55,26}},
                                                     color={0,0,127}));
        connect(conversion.m_dot_prim_is, m_dot_prim_is.y)
          annotation (Line(points={{14,1.77778},{50,1.77778},{50,0},{55,0}},
                                                                 color={0,0,127}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
              Ellipse(lineColor = {75,138,73},
                      fillColor={255,255,255},
                      fillPattern = FillPattern.Solid,
                      extent={{-100,-100},{100,100}}),
              Polygon(lineColor = {0,0,255},
                      fillColor = {75,138,73},
                      pattern = LinePattern.None,
                      fillPattern = FillPattern.Solid,
                      points={{-36,60},{64,0},{-36,-60},{-36,60}})}),  Diagram(
              coordinateSystem(preserveAspectRatio=false)));
      end Test_Conversion;

      model Test_heat_transfer_station_production
        Modelica.Blocks.Sources.RealExpression T_house(y=273.15 + 70)
          annotation (Placement(transformation(extent={{-92,74},{-72,94}})));
        Modelica.Blocks.Sources.IntegerExpression pi(y=1)
          annotation (Placement(transformation(extent={{-92,44},{-72,64}})));
        Modelica.Blocks.Sources.IntegerExpression mu(y=1)
          annotation (Placement(transformation(extent={{-92,30},{-72,50}})));
        Modelica.Blocks.Sources.RealExpression u_pump(y=0.5)
          annotation (Placement(transformation(extent={{-92,12},{-72,32}})));
        Modelica.Blocks.Sources.RealExpression kappa(y=0.8)
          annotation (Placement(transformation(extent={{-92,-2},{-72,18}})));
        Modelica.Blocks.Sources.RealExpression flow_house(y=5)
          annotation (Placement(transformation(extent={{-92,62},{-72,82}})));
        Modelica.Blocks.Sources.Ramp ramp(
          height=1,
          duration=600,
          offset=0,
          startTime=200)
          annotation (Placement(transformation(extent={{-92,-32},{-72,-12}})));
        HTS heat_transfer_station1(
          redeclare Fluid.Pumps.Data.Pumps.QuadraticCharacteristic feedinPer,
          energyDynamics_feedPump=Modelica.Fluid.Types.Dynamics.DynamicFreeInitial,
          use_inputFilter_feedPump=true,
          init_feedPump=Modelica.Blocks.Types.Init.InitialOutput,
          use_inputFilter_conVal=true,
          init_conVal=Modelica.Blocks.Types.Init.InitialOutput,
          ambient_temperature=system.T_ambient,
          energyDynamics_pumpsSec=Modelica.Fluid.Types.Dynamics.DynamicFreeInitial,
          use_inputFilter_pumpsSec=true)
          annotation (Placement(transformation(extent={{-8,40},{22,76}})));

        Modelica.Fluid.Vessels.ClosedVolume volume(
          T_start=318.15,
          use_portsData=false,                     V=1, nPorts=2,
        redeclare final package Medium = Media.Water)
          annotation (Placement(transformation(extent={{2,-24},{22,-44}})));
        inner Modelica.Fluid.System system(T_ambient=285.15)
          annotation (Placement(transformation(extent={{60,62},{80,82}})));
        Fluid.Valves.TwoWayEqualPercentage valve_for_test(
          m_flow_nominal=30.074213*0.001/60,
          redeclare final package Medium = Media.Water,
          final CvData=ProsNet.Fluid.Types.CvTypes.Kv,
          final Kv=2.5,
          final use_inputFilter=true,
          final riseTime=5,
          final init=Modelica.Blocks.Types.Init.InitialOutput,
          final y_start=0,
          final l=2e-3) annotation (Placement(transformation(
              extent={{10,10},{-10,-10}},
              rotation=-90,
              origin={36,-4})));
        Fluid.Sources.Boundary_pT bou(nPorts=1, redeclare final package Medium =
              Media.Water)
          annotation (Placement(transformation(extent={{106,2},{86,22}})));
      equation
        connect(T_house.y, heat_transfer_station1.T_sec_in_set) annotation (Line(
              points={{-71,84},{-14,84},{-14,72},{-8,72}}, color={0,0,127}));
        connect(flow_house.y, heat_transfer_station1.V_dot_sec_set) annotation (Line(
              points={{-71,72},{-16,72},{-16,68},{-8,68}}, color={0,0,127}));
        connect(pi.y, heat_transfer_station1.pi) annotation (Line(points={{-71,54},{-16,
                54},{-16,62},{-8,62}}, color={255,127,0}));
        connect(mu.y, heat_transfer_station1.mu) annotation (Line(points={{-71,40},{-14,
                40},{-14,58},{-8,58}}, color={255,127,0}));
        connect(u_pump.y, heat_transfer_station1.u_set) annotation (Line(points={{-71,
                22},{-66,22},{-66,52},{-60,52},{-60,54},{-8,54}}, color={0,0,127}));
        connect(kappa.y, heat_transfer_station1.kappa_set) annotation (Line(points={{-71,
                8},{-58,8},{-58,50},{-8,50}}, color={0,0,127}));
        connect(heat_transfer_station1.hot_prim, volume.ports[1])
          annotation (Line(points={{-3.5,39.8},{-3.5,-24},{11,-24}},
                                                               color={0,127,255}));
        connect(valve_for_test.port_b, heat_transfer_station1.cold_prim) annotation (
            Line(points={{36,6},{36,34},{17.5,34},{17.5,40}},
                                                            color={0,127,255}));
        connect(valve_for_test.port_a, volume.ports[2])
          annotation (Line(points={{36,-14},{36,-24},{13,-24}}, color={0,127,255}));
        connect(ramp.y, valve_for_test.y) annotation (Line(points={{-71,-22},{0,-22},{
                0,-4},{24,-4}}, color={0,0,127}));
        connect(bou.ports[1], valve_for_test.port_b)
          annotation (Line(points={{86,12},{36,12},{36,6}}, color={0,127,255}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
              Ellipse(lineColor = {75,138,73},
                      fillColor={255,255,255},
                      fillPattern = FillPattern.Solid,
                      extent={{-100,-100},{100,100}}),
              Polygon(lineColor = {0,0,255},
                      fillColor = {75,138,73},
                      pattern = LinePattern.None,
                      fillPattern = FillPattern.Solid,
                      points={{-36,60},{64,0},{-36,-60},{-36,60}})}),  Diagram(
              coordinateSystem(preserveAspectRatio=false)),
          experiment(
            StopTime=1000,
            Interval=1,
            __Dymola_Algorithm="Dassl"));
      end Test_heat_transfer_station_production;

      model Test_heat_transfer_station_consumption
        Modelica.Blocks.Sources.RealExpression T_house(y=273.15 + 45)
          annotation (Placement(transformation(extent={{-92,74},{-72,94}})));
        Modelica.Blocks.Sources.IntegerExpression pi(y=1)
          annotation (Placement(transformation(extent={{-92,44},{-72,64}})));
        Modelica.Blocks.Sources.IntegerExpression mu(y=-1)
          annotation (Placement(transformation(extent={{-92,30},{-72,50}})));
        Modelica.Blocks.Sources.RealExpression u_pump(y=0.5)
          annotation (Placement(transformation(extent={{-92,12},{-72,32}})));
        Modelica.Blocks.Sources.RealExpression kappa(y=0.8)
          annotation (Placement(transformation(extent={{-92,-2},{-72,18}})));
        Modelica.Blocks.Sources.RealExpression flow_house(y=5)
          annotation (Placement(transformation(extent={{-92,62},{-72,82}})));
        Modelica.Blocks.Sources.Ramp ramp(
          height=1,
          duration=600,
          offset=0,
          startTime=200)
          annotation (Placement(transformation(extent={{-92,-32},{-72,-12}})));
        HTS heat_transfer_station1(
          redeclare Fluid.Pumps.Data.Pumps.QuadraticCharacteristic feedinPer,
          energyDynamics_feedPump=Modelica.Fluid.Types.Dynamics.DynamicFreeInitial,
          use_inputFilter_feedPump=true,
          init_feedPump=Modelica.Blocks.Types.Init.InitialOutput,
          use_inputFilter_conVal=true,
          init_conVal=Modelica.Blocks.Types.Init.InitialOutput,
          ambient_temperature=system.T_ambient,
          energyDynamics_pumpsSec=Modelica.Fluid.Types.Dynamics.DynamicFreeInitial,
          use_inputFilter_pumpsSec=true,
          energyDynamics_cv=Modelica.Fluid.Types.Dynamics.SteadyState)
          annotation (Placement(transformation(extent={{-8,40},{22,76}})));

        Modelica.Fluid.Vessels.ClosedVolume volume(
          T_start=338.15,
          use_portsData=false,                     V=1, nPorts=2,
        redeclare final package Medium = Media.Water)
          annotation (Placement(transformation(extent={{2,-30},{22,-50}})));
        inner Modelica.Fluid.System system(T_ambient=285.15)
          annotation (Placement(transformation(extent={{62,62},{82,82}})));
        Fluid.Sources.Boundary_pT bou(          redeclare final package Medium =
              Media.Water, nPorts=1)
          annotation (Placement(transformation(extent={{106,4},{86,24}})));
        Fluid.Pumps.SpeedControlled_y pump_prim_prod(
          redeclare final package Medium = Media.Water,
          final energyDynamics=Modelica.Fluid.Types.Dynamics.DynamicFreeInitial,
          final tau=1,
          redeclare final Fluid.Pumps.Data.Pumps.IMP.NMTPlus152025to40 per,
          final use_inputFilter=true,
          final riseTime=5,
          final init=Modelica.Blocks.Types.Init.InitialOutput,
          final y_start=0)                annotation (Placement(transformation(
              extent={{10,10},{-10,-10}},
              rotation=-90,
              origin={40,2})));
      equation
        connect(T_house.y, heat_transfer_station1.T_sec_in_set) annotation (Line(
              points={{-71,84},{-14,84},{-14,72},{-8,72}}, color={0,0,127}));
        connect(pi.y, heat_transfer_station1.pi) annotation (Line(points={{-71,54},{-16,
                54},{-16,62},{-8,62}}, color={255,127,0}));
        connect(mu.y, heat_transfer_station1.mu) annotation (Line(points={{-71,40},{-14,
                40},{-14,58},{-8,58}}, color={255,127,0}));
        connect(u_pump.y, heat_transfer_station1.u_set) annotation (Line(points={{-71,
                22},{-66,22},{-66,52},{-60,52},{-60,54},{-8,54}}, color={0,0,127}));
        connect(kappa.y, heat_transfer_station1.kappa_set) annotation (Line(points={{-71,
                8},{-58,8},{-58,50},{-8,50}}, color={0,0,127}));
        connect(heat_transfer_station1.hot_prim, volume.ports[1])
          annotation (Line(points={{-3.5,39.8},{-3.5,-30},{11,-30}},
                                                               color={0,127,255}));
        connect(volume.ports[2], pump_prim_prod.port_a) annotation (Line(points={{13,-30},
                {22,-30},{22,-16},{40,-16},{40,-8}}, color={0,127,255}));
        connect(heat_transfer_station1.cold_prim, pump_prim_prod.port_b) annotation (
            Line(points={{17.5,40},{17.5,18},{40,18},{40,12}},
                                                             color={0,127,255}));
        connect(ramp.y, pump_prim_prod.y) annotation (Line(points={{-71,-22},{10,-22},
                {10,2},{28,2}}, color={0,0,127}));
        connect(bou.ports[1], pump_prim_prod.port_b)
          annotation (Line(points={{86,14},{54,14},{54,12},{40,12}},
                                                             color={0,127,255}));
        connect(flow_house.y, heat_transfer_station1.V_dot_sec_set) annotation (
            Line(points={{-71,72},{-16,72},{-16,68},{-8,68}}, color={0,0,127}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
              Ellipse(lineColor = {75,138,73},
                      fillColor={255,255,255},
                      fillPattern = FillPattern.Solid,
                      extent={{-100,-100},{100,100}}),
              Polygon(lineColor = {0,0,255},
                      fillColor = {75,138,73},
                      pattern = LinePattern.None,
                      fillPattern = FillPattern.Solid,
                      points={{-36,60},{64,0},{-36,-60},{-36,60}})}),  Diagram(
              coordinateSystem(preserveAspectRatio=false)),
          experiment(
            StopTime=1000,
            Interval=1,
            __Dymola_Algorithm="Dassl"));
      end Test_heat_transfer_station_consumption;
    end Validation;

    annotation (conversion(noneFromVersion=""));
  end New_Substation;

  package Substation_idealProsumer
    "Models that are currently under development"

    package new_prosumer_models

      model heat_transfer_station

        replaceable package Medium_prim = ProsNet.Media.Water;
        replaceable package Medium_sec = ProsNet.Media.Water;

        extends ProsNet.Prosumers.BaseClasses.PrimarySideParameters;
        extends ProsNet.Prosumers.SecondarySides.BaseClasses.PumpsPairDynParam;
        extends
          ProsNet.Prosumers.SecondarySides.BaseClasses.ControlVolumeDynParam;

        Modelica.Blocks.Interfaces.RealVectorInput contr_vars_real[6]
      annotation (Placement(transformation(extent={{-222,-20},{-182,20}})));

        Modelica.Blocks.Interfaces.RealVectorOutput states[8]
      annotation (Placement(transformation(extent={{180,-20},{220,20}})));

        Real T_sec_in_set(unit="K", displayUnit="degC") "K"
      annotation (Placement(transformation(extent={{-180,80},{-140,120}})));
        Real V_dot_sec_set(unit="l/min", displayUnit="l/min") "l/min"
      annotation (Placement(transformation(extent={{-180,40},{-140,80}})));
        Real pi
      annotation (Placement(transformation(extent={{-180,0},{-140,40}})));
        Real mu
      annotation (Placement(transformation(extent={{-180,-40},{-140,0}})));
        Real u_set
      annotation (Placement(transformation(extent={{-180,-80},{-140,-40}})));
        Real kappa_set
      annotation (Placement(transformation(extent={{-180,-120},{-140,-80}})));

      Real T_prim_hot(unit="K", displayUnit="degC")                                 "K"
                                                     annotation (Placement(
          transformation(
          extent={{-20,-20},{20,20}},
          rotation=180,
          origin={160,140})));

        Real T_prim_cold(unit="K", displayUnit="degC") "K"
                                                      annotation (Placement(
          transformation(
          extent={{-20,-20},{20,20}},
          rotation=180,
          origin={160,100})));

        Real T_sec_hot(
          unit="K",
          displayUnit="degC",
      start=45 + 273.15)                                                         "K"
                                                    annotation (Placement(
          transformation(
          extent={{-20,-20},{20,20}},
          rotation=180,
          origin={160,60})));

        Real T_sec_cold(
          unit="K",
          displayUnit="degC",
      start=30 + 273.15)                                                           "K"
                                                     annotation (Placement(
          transformation(
          extent={{-20,-20},{20,20}},
          rotation=180,
          origin={160,20})));

        Real V_dot_prim(unit="l/min", displayUnit="l/min") "l/min"
                                                     annotation (Placement(
          transformation(
          extent={{-20,-20},{20,20}},
          rotation=180,
          origin={160,-20})));

        Real V_dot_sec(unit="l/min", displayUnit="l/min") "l/min"
                                                    annotation (Placement(
          transformation(
          extent={{20,-20},{-20,20}},
          rotation=0,
          origin={160,-60})));

        Real Q_dot_is(unit="kW", displayUnit="kW")
      "kW" annotation (Placement(transformation(
          extent={{20,-20},{-20,20}},
          rotation=0,
          origin={160,-100})));

        Real Delta_p_prim(unit="Pa", displayUnit="bar") annotation (
        Placement(transformation(
          extent={{-20,-20},{20,20}},
          rotation=180,
          origin={160,-140})));

        Real cp_prim;

        ProsNet.Fluid.HeatExchangers.LiquidToLiquid heat_exchanger(
          redeclare package Medium2 = Medium_sec,
          redeclare package Medium1 = Medium_prim,
          m1_flow_nominal=m_flow_nominal_1,
          m2_flow_nominal=m_flow_nominal_2,
          show_T=true,
          dp1_nominal=dp1_nominal,
          dp2_nominal=dp2_nominal,
          Q_flow_nominal=Q_flow_nominal,
          T_a1_nominal=338.15,
          T_a2_nominal=313.15)
          annotation (Placement(transformation(extent={{30,12},{50,-8}})));

        ProsNet.Fluid.Pumps.FlowControlled_m_flow pump_sec_cons(
          redeclare package Medium = Medium_sec,
          final energyDynamics=energyDynamics_pumpsSec,
          T_start=313.15,
          final tau=tau_pumpsSec,
          final m_flow_nominal=m_flow_nominal_2,
          final use_inputFilter=use_inputFilter_pumpsSec,
          final riseTime=riseTime_pumpsSec,
          final dp_nominal=dp2_nominal,
          final m_flow_start=m_flow_start_pumpsSec,
          final y_start=y_start_pumpsSec) annotation (Placement(transformation(
              extent={{-10,10},{10,-10}},
              rotation=270,
              origin={60,70})));
        ProsNet.Fluid.Pumps.FlowControlled_m_flow pump_sec_prod(
          redeclare package Medium = Medium_sec,
          final energyDynamics=energyDynamics_pumpsSec,
          T_start=313.15,
          final tau=tau_pumpsSec,
          final m_flow_nominal=m_flow_nominal_2,
          final use_inputFilter=use_inputFilter_pumpsSec,
          final riseTime=riseTime_pumpsSec,
          final dp_nominal=dp2_nominal,
          final m_flow_start=m_flow_start_pumpsSec,
          final y_start=y_start_pumpsSec) annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=90,
              origin={100,42})));
        ProsNet.Fluid.FixedResistances.CheckValve cheVa_sec_cons(
          m_flow_nominal=m_flow_nominal_2,
          redeclare final package Medium = Medium_sec,
          final CvData=ProsNet.Fluid.Types.CvTypes.Kv,
          final Kv=Kv_cheVal,
          final l=l_cheVal) annotation (Placement(transformation(
              extent={{-10,10},{10,-10}},
              rotation=270,
              origin={60,42})));
        ProsNet.Fluid.FixedResistances.CheckValve cheVal_sec_prod(
          m_flow_nominal=m_flow_nominal_2,
          redeclare final package Medium = Medium_sec,
          final CvData=ProsNet.Fluid.Types.CvTypes.Kv,
          final Kv=Kv_cheVal,
          final l=l_cheVal) annotation (Placement(transformation(
              extent={{10,-10},{-10,10}},
              rotation=270,
              origin={100,70})));
        ProsNet.Fluid.Valves.TwoWayEqualPercentage valve_prim_cons(
          m_flow_nominal=m_flow_nominal_1,
          redeclare final package Medium = Medium_prim,
          final CvData=ProsNet.Fluid.Types.CvTypes.Kv,
          final Kv=Kv_conVal,
          final use_inputFilter=use_inputFilter_conVal,
          final riseTime=riseTime_conVal,
          final init=init_conVal,
          final y_start=y_start_conVal,
          final l=l_conVal) annotation (Placement(transformation(
              extent={{10,10},{-10,-10}},
              rotation=-90,
              origin={60,-40})));
        ProsNet.Fluid.Pumps.SpeedControlled_y pump_prim_prod(
          redeclare final package Medium = Medium_prim,
          final energyDynamics=energyDynamics_feedPump,
          T_start=313.15,
          final tau=tau_feedPump,
          final per=feedinPer,
          final use_inputFilter=use_inputFilter_feedPump,
          final riseTime=riseTime_feedPump,
          final init=init_feedPump,
          final y_start=y_start_feedPump) annotation (Placement(transformation(
              extent={{10,10},{-10,-10}},
              rotation=-90,
              origin={100,-70})));
        ProsNet.Fluid.FixedResistances.CheckValve cheVal_prim_prod(
          m_flow_nominal=m_flow_nominal_1,
          redeclare final package Medium = Medium_prim,
          final CvData=ProsNet.Fluid.Types.CvTypes.Kv,
          final Kv=Kv_cheVal,
          final l=l_cheVal) annotation (Placement(transformation(
              extent={{10,10},{-10,-10}},
              rotation=-90,
              origin={100,-40})));
        ProsNet.Fluid.FixedResistances.CheckValve cheVal_prim_cons(
          m_flow_nominal=m_flow_nominal_1,
          redeclare final package Medium = Medium_prim,
          final CvData=ProsNet.Fluid.Types.CvTypes.Kv,
          final Kv=Kv_cheVal,
          final l=l_cheVal) annotation (Placement(transformation(
              extent={{10,-10},{-10,10}},
              rotation=90,
              origin={62,-70})));
        Modelica.Fluid.Interfaces.FluidPort_b cold_prim(redeclare final package
            Medium =
              Medium_prim)
      annotation (Placement(transformation(extent={{130,-190},{150,-170}})));
        Modelica.Fluid.Interfaces.FluidPort_a hot_prim(redeclare final package
            Medium =
              Medium_prim)
      annotation (Placement(transformation(extent={{-150,-192},{-130,-172}})));
        heat_source_sink_ideal ideal_house(
      energyDynamics_cv=Modelica.Fluid.Types.Dynamics.FixedInitial,
      tau_cv=10,
      T_start_cv=313.15)
      annotation (Placement(transformation(extent={{18,120},{62,152}})));
        Conversion conversion
      annotation (Placement(transformation(extent={{-100,-4},{-52,66}})));
        Modelica.Fluid.Sensors.MassFlowRate m_dot_sens_prim(
      redeclare package Medium = Medium_prim, allowFlowReversal=true)
                                              annotation (Placement(
          transformation(
          extent={{10,-10},{-10,10}},
          rotation=90,
          origin={-20,-72})));
        Modelica.Fluid.Sensors.TemperatureTwoPort T_sens_prim_hot(
      redeclare package Medium = Medium_prim, allowFlowReversal=true) annotation (
        Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=90,
          origin={-20,-110})));
        Modelica.Fluid.Sensors.TemperatureTwoPort T_sens_prim_cold(
      redeclare package Medium = Medium_prim, allowFlowReversal=true)
                                              annotation (
        Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=-90,
          origin={80,-110})));
        ProsNet.Fluid.Sources.Boundary_pT bou(
          redeclare package Medium = Medium_sec,
          T=313.15,
          nPorts=1)
          annotation (Placement(transformation(extent={{128,118},{108,138}})));

        ProsNet.Fluid.Sensors.RelativePressure pressureDifference(redeclare
            package Medium = Medium_prim)
          annotation (Placement(transformation(extent={{4,-164},{24,-144}})));

        ProsNet.Fluid.Pipes.InsulatedPipe_plug pipe_prim_hot(
          T_amb=ambient_temperature,
          R_ins=R_ins_transferpipe,
          length=length_transfer_pipe_tot/2,
          diameter=d_transferpipe/2,
          zeta=zeta_transferstation/2) annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=90,
              origin={-20,-138})));
        ProsNet.Fluid.Pipes.InsulatedPipe_plug pipe_prim_cold(
          T_amb=ambient_temperature,
          R_ins=R_ins_transferpipe,
          length=length_transfer_pipe_tot/2,
          diameter=d_transferpipe/2,
          zeta=zeta_transferstation/2) annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=-90,
              origin={80,-140})));
      equation

        connect(
            contr_vars_real[1], conversion.T_sec_in_set);
        connect(
            contr_vars_real[2], conversion.V_dot_sec_set);
        connect(
            contr_vars_real[3], conversion.pi);
        connect(
            contr_vars_real[4], conversion.mu);
        connect(
            contr_vars_real[5], conversion.u_set);
        connect(
            contr_vars_real[6], conversion.kappa_set);

        T_sec_in_set = contr_vars_real[1];
        V_dot_sec_set = contr_vars_real[2];
        pi = contr_vars_real[3];
        mu = contr_vars_real[4];
        u_set = contr_vars_real[5];
        kappa_set = contr_vars_real[6];

        cp_prim = 4200;
        states[7] = m_dot_sens_prim.port_a.m_flow*cp_prim*(T_sens_prim_hot.T -
          T_sens_prim_cold.T)/1000;

        connect(
            heat_exchanger.port_b1, valve_prim_cons.port_b)
      annotation (Line(points={{50,-4},{60,-4},{60,-30}}, color={0,127,255}));
        connect(
            cheVal_prim_prod.port_b,heat_exchanger. port_b1)
      annotation (Line(points={{100,-30},{100,-4},{50,-4}},
                                                          color={0,127,255}));
        connect(
            cheVa_sec_cons.port_b,heat_exchanger. port_a2)
      annotation (Line(points={{60,32},{60,8},{50,8}}, color={0,127,255}));
        connect(
            pump_sec_prod.port_a,heat_exchanger. port_a2)
      annotation (Line(points={{100,32},{100,8},{50,8}},
                                                       color={0,127,255}));
        connect(
            pump_sec_cons.port_b, cheVa_sec_cons.port_a)
      annotation (Line(points={{60,60},{60,52}}, color={0,127,255}));
        connect(
            cheVal_sec_prod.port_a, pump_sec_prod.port_b)
      annotation (Line(points={{100,60},{100,52}},
                                                 color={0,127,255}));
        connect(
            valve_prim_cons.port_a, cheVal_prim_cons.port_a)
      annotation (Line(points={{60,-50},{60,-60},{62,-60}},
                                                   color={0,127,255}));
        connect(
            cheVal_prim_prod.port_a, pump_prim_prod.port_b)
      annotation (Line(points={{100,-50},{100,-60}},
                                                   color={0,127,255}));
        connect(
            conversion.T_sec_in, ideal_house.T_set) annotation (Line(points={{-66.4,
            66},{-70,66},{-70,158},{40,158},{40,152}}, color={0,0,127}));
        connect(
            conversion.pump_contr, pump_prim_prod.y) annotation (Line(points={{-52,
                20.7059},{-38,20.7059},{-38,20},{-24,20},{-24,-24},{82,-24},{82,
                -70},{88,-70}},
                      color={0,0,127}));
        connect(
            conversion.valve_contr, valve_prim_cons.y) annotation (Line(points={{-52,
                12.4706},{-42,12.4706},{-42,12},{-32,12},{-32,-40},{48,-40}},
                                                                         color={0,
            0,127}));
        connect(
            conversion.m_dot_cons, pump_sec_cons.m_flow_in) annotation (Line(
          points={{-52,37.1765},{42,37.1765},{42,70},{48,70}},
                                                            color={0,0,127}));
        connect(
            conversion.m_dot_prod, pump_sec_prod.m_flow_in) annotation (Line(
          points={{-52,45.4118},{-12,45.4118},{-12,46},{28,46},{28,24},{82,24},
                {82,42},{88,42}},
                             color={0,0,127}));

        connect(
            ideal_house.m_dot_sec_is, conversion.m_dot_sec_is);

        connect(
            m_dot_sens_prim.m_flow, conversion.m_dot_prim_is);
        connect(
            cheVal_prim_cons.port_b, T_sens_prim_cold.port_a) annotation (Line(
          points={{62,-80},{62,-90},{80,-90},{80,-100}}, color={0,127,255}));
        connect(
            pump_prim_prod.port_a, T_sens_prim_cold.port_a) annotation (Line(
          points={{100,-80},{100,-90},{80,-90},{80,-100}},
                                                         color={0,127,255}));

        connect(
            ideal_house.port_cold, cheVal_sec_prod.port_b) annotation (Line(
          points={{53.2,120},{54,120},{54,100},{80,100},{80,88},{100,88},{100,80}},
          color={0,127,255}));
        connect(
            ideal_house.port_cold, pump_sec_cons.port_a) annotation (Line(points={{53.2,
            120},{54,120},{54,100},{80,100},{80,88},{60,88},{60,80}},       color=
           {0,127,255}));
        connect(
            ideal_house.port_hot, heat_exchanger.port_b2) annotation (Line(points={{26.8,
            120},{32,120},{32,100},{0,100},{0,8},{30,8}},             color={0,127,
            255}));
        connect(
            bou.ports[1], cheVal_sec_prod.port_b)
      annotation (Line(points={{108,128},{100,128},{100,80}},
                                                           color={0,127,255}));
        connect(
            hot_prim, pressureDifference.port_a) annotation (Line(points={{-140,-182},
            {-140,-154},{4,-154}},  color={0,127,255}));
        connect(
            pressureDifference.port_b, cold_prim) annotation (Line(points={{24,-154},
            {68,-154},{68,-166},{140,-166},{140,-180}},
                                  color={0,127,255}));
        connect(
            heat_exchanger.port_a1, m_dot_sens_prim.port_a)
      annotation (Line(points={{30,-4},{-20,-4},{-20,-62}},  color={0,127,255}));
        connect(
            m_dot_sens_prim.port_b, T_sens_prim_hot.port_b)
      annotation (Line(points={{-20,-82},{-20,-100}}, color={0,127,255}));

        connect(
            T_sens_prim_hot.T, states[1]);
        connect(
            T_sens_prim_cold.T, states[2]);
        connect(
            ideal_house.T_sec_hot, states[3]);
        connect(
            ideal_house.T_sec_cold, states[4]);
        connect(
            conversion.V_dot_prim_is, states[5]);
        connect(
            conversion.V_dot_sec_is, states[6]);
        connect(
            pressureDifference.p_rel, states[8]);

        T_prim_hot = T_sens_prim_hot.T;
        T_prim_cold = T_sens_prim_cold.T;
        T_sec_hot = ideal_house.T_sec_hot;
        T_sec_cold = ideal_house.T_sec_cold;
        V_dot_prim = conversion.V_dot_prim_is;
        V_dot_sec = conversion.V_dot_sec_is;
        Q_dot_is = states[7];
        Delta_p_prim = pressureDifference.p_rel;

        connect(
            pipe_prim_hot.port_b, T_sens_prim_hot.port_a)
      annotation (Line(points={{-20,-128},{-20,-120}}, color={0,127,255}));
        connect(
            pipe_prim_hot.port_a, hot_prim) annotation (Line(points={{-20,-148},{-20,-154},
            {-140,-154},{-140,-182}}, color={0,127,255}));
        connect(
            T_sens_prim_cold.port_b, pipe_prim_cold.port_a)
      annotation (Line(points={{80,-120},{80,-130}}, color={0,127,255}));
        connect(
            pipe_prim_cold.port_b, cold_prim) annotation (Line(points={{80,-150},{80,-166},
            {140,-166},{140,-180}}, color={0,127,255}));
        annotation (Diagram(coordinateSystem(extent={{-200,-180},{200,180}})), Icon(
          coordinateSystem(extent={{-200,-180},{200,180}}), graphics={
          Line(
            points={{-140,-98},{-140,-170}},
            color={238,46,47},
            thickness=0.5),
          Line(
            points={{138,-100},{138,-168}},
            color={28,108,200},
            thickness=0.5),
          Rectangle(
            extent={{-38,30},{48,-48}},
            lineColor={0,0,0},
            lineThickness=1),
          Bitmap(extent={{-106,88},{108,176}}, fileName=
                "modelica://ProsNet/../../../../Downloads/noun-home-121812.svg"),
          Line(
            points={{32,86},{32,28}},
            color={28,108,200},
            thickness=0.5),
          Line(
            points={{-22,86},{-22,28}},
            color={238,46,47},
            thickness=0.5),
          Line(
            points={{138,-100},{38,-100}},
            color={28,108,200},
            thickness=0.5),
          Line(
            points={{-142,-100},{-24,-100}},
            color={238,46,47},
            thickness=0.5),
          Line(
            points={{-24,-52},{-24,-102}},
            color={238,46,47},
            thickness=0.5),
          Line(
            points={{36,-50},{36,-102}},
            color={28,108,200},
            thickness=0.5),
          Rectangle(extent={{-200,180},{200,-180}}, lineColor={0,0,0})}));
      end heat_transfer_station;

      model heat_source_sink_ideal

        extends
          ProsNet.Prosumers.SecondarySides.BaseClasses.ControlVolumeDynParam;

        ProsNet.Fluid.HeatExchangers.ControlVolume_T control_volume(
          redeclare final package Medium = ProsNet.Media.Water,
          allowFlowReversal=true,
          m_flow_nominal=0.5,
          tau=tau_cv,
          T_start=T_start_cv,
          energyDynamics=energyDynamics_cv)
          annotation (Placement(transformation(extent={{-28,-26},{28,28}})));
        Modelica.Blocks.Interfaces.RealInput T_set(unit="K", displayUnit="degC")
          "Temperature set point" annotation (Placement(transformation(
              extent={{-20,-20},{20,20}},
              rotation=-90,
              origin={0,100})));
        Modelica.Fluid.Interfaces.FluidPort_a port_hot(redeclare package Medium =
              ProsNet.Media.Water)
          annotation (Placement(transformation(extent={{-70,-110},{-50,-90}})));
        Modelica.Fluid.Interfaces.FluidPort_b port_cold(redeclare package
            Medium =
              ProsNet.Media.Water)
          annotation (Placement(transformation(extent={{50,-110},{70,-90}})));
        Modelica.Blocks.Interfaces.RealOutput m_dot_sec_is "kg/s" annotation (
            Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=-90,
              origin={0,-98})));
        Modelica.Blocks.Interfaces.RealOutput T_sec_hot annotation (Placement(
              transformation(
              extent={{-10,-10},{10,10}},
              rotation=-90,
              origin={-80,-100})));
        Modelica.Blocks.Interfaces.RealOutput T_sec_cold annotation (Placement(
              transformation(
              extent={{-10,-10},{10,10}},
              rotation=-90,
              origin={80,-100})));
        Modelica.Fluid.Sensors.TemperatureTwoPort T_sens_hot(redeclare package
            Medium =
              ProsNet.Media.Water)
                           annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=90,
              origin={-60,-50})));
        Modelica.Fluid.Sensors.TemperatureTwoPort T_sens_cold(redeclare package
            Medium = ProsNet.Media.Water)
                                  annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=-90,
              origin={60,-50})));
        Modelica.Fluid.Sensors.MassFlowRate massFlowRate(redeclare package
            Medium = ProsNet.Media.Water)
                                  annotation (Placement(
              transformation(
              extent={{-10,10},{10,-10}},
              rotation=90,
              origin={-60,-16})));
      equation
        connect(T_set, control_volume.TSet) annotation (Line(points={{0,100},{0,62},
                {-34,62},{-34,28},{-33.6,28},{-33.6,22.6}}, color={0,0,127}));
        connect(T_sens_hot.port_a, port_hot)
          annotation (Line(points={{-60,-60},{-60,-100}}, color={0,127,255}));
        connect(T_sens_hot.T, T_sec_hot) annotation (Line(points={{-71,-50},{-80,-50},
                {-80,-100}}, color={0,0,127}));
        connect(control_volume.port_b, T_sens_cold.port_a)
          annotation (Line(points={{28,1},{60,1},{60,-40}}, color={0,127,255}));
        connect(T_sens_cold.port_b, port_cold)
          annotation (Line(points={{60,-60},{60,-100}}, color={0,127,255}));
        connect(T_sens_cold.T, T_sec_cold)
          annotation (Line(points={{71,-50},{80,-50},{80,-100}}, color={0,0,127}));
        connect(massFlowRate.port_b, control_volume.port_a)
          annotation (Line(points={{-60,-6},{-60,1},{-28,1}}, color={0,127,255}));
        connect(massFlowRate.port_a, T_sens_hot.port_b)
          annotation (Line(points={{-60,-26},{-60,-40}}, color={0,127,255}));
        connect(massFlowRate.m_flow, m_dot_sec_is) annotation (Line(points={{-49,-16},
                {-34,-16},{-34,-84},{0,-84},{0,-98}}, color={0,0,127}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
              Rectangle(
                extent={{-100,40},{100,-100}},
                lineColor={0,0,0},
                lineThickness=1),
              Polygon(
                points={{100,40},{60,100},{-60,100},{-100,40},{100,40}},
                lineColor={0,0,0},
                lineThickness=1),
              Text(
                extent={{-118,88},{118,2}},
                textColor={0,0,0},
                textString="ideal
"),           Bitmap(
                extent={{-100,-78},{0,20}},
                imageSource="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",
                fileName="modelica://ProsNet/../../../../Downloads/noun_Fire_3890703.png"),
              Bitmap(
                extent={{0,-80},{98,20}},
                imageSource="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"
                     + "W7+21N+WOvvaVm/V3B6SpKE8ClhHOGj8Fthxntf/NTMHme9Hj8cHngXAnwM/IQyBWBa9biFwHHAFsBLYAPyScEZ5rwHv+wTg/wI/7C23EbgfuBI4CXhozjKDhmbkhd+ytS0B/h74VW+Zu4GvAgdm6igbeBcCbyJs7/WEs+5XA28DtoqW2Ylwxm0LYfs8smDdH4/W//oh63kUcDqwovf+G4BbgX8FDpmn/sXAe4CfMrN9vgw8dcD7ldnfbakz+3n4097y64F7gX8Hnl7wfoM+Sx+Inr+ZMB5/GKNus6I6xukXwLaEbbaC8DleS/iW6bgh37PM/pMkTYlTmDlovGWI1+8FvK/X3hs9Hh94Ps3sYLms95qHAZdknovbSuAFOe95HPC7ActtAX7B3FA6SuAtW9thwH8WLLMJeENm2wyrv8zPCOG5qK5rgIdHy30xeu5tOetdQPjDZguwhuLgGDuA4j7223sK6v8pcG7BMmvID0Bl93db6ow/Dx8sWG4tsG/OexZ9lk6KnrsB+L2cZfOU2WZFdYzTr8cw+5uYbDttnvcss/8kSVPkKmYODLuPsZ7+Oh7s/VwOvIJwQH08IWid33tuM/BZ4EjgIOAE4I7ec+uYfUbmCYSzNVsIZ88+0VvuAOAY4HvRe38uU9O+vXZL9Jp9o9ZXtrZdgHuidV8JHN1b7s+A65h94C0TePvtIsIY64MIQ0niPl0QLff8TD1Zz46e/+KQtayIljkd+MPeek4inBHsb7enFNS/Efir3jIvYPZn7tzMe42zv9tSZ//xdb11FL3n15kr77N0YvT4j4DdcpYrUmabFdVRtl/b9OruP3854d+Og4HXEH6PNs/znqPuP0nSlHmAcEC4a8z1xAeeK5g7i8NLouePz1l+CbCq9/yl0eMnRMt9JGe5RzL7YJ9nvjG8ZWt7f7TcJcy9oHEHZofesoH3XMKwhthuwG+i1xzWe3wh8Ovo8SWZ5c6InjtiyFr647ZvzXnubcD1vRaP6Y7rf3tmmT0JZ7+3EKa8i42zv9tSZ9n3jJftf5beET32PcIfYaMos83y6ogfG7Vfb4yWu4gwY0xse4p/j8bZlpKkKdI/INw85nriA8/BOc9/jZkDT/aA1vfZaB39oLYzsHev7ZCzzAHkHwhj8wXesrV9N3ps/4LlXjFEfXni7fnkgtfEQeGM6PFTo8dPjh5fxMzZ6tso7mtWvP0+zuDxzNn61xDO4GX9ivx9Ms7+bkudZd8zXvZGwvzS/f9fQRiWM6oy2yxbR/axUft1afT4ftmFel4+4feUJE2ZSZ/hXcncs5EwE7SGbcdkll9AuOnFywljhz9JmFEinjWibOAtW9t9UZ8XFKx7tyHqy9Nf5t4Br9kret0V0eOPY+Yr4Ouix58bvf60EWp5JbO/Uu7/gXQO4Yz4YwbU/7OCdc63T8rs77bUOc579h/Pjh2+h+KLFAcps83iOvLC56j9Gub3aNcJv6ckacqMOoZ3R+D/9NpfRo/313FTwXLz3dAi2/5nb7kFzB5HG7eVhPGA4wbesrX1x3AW9RnCMIdxAu+w674u89xl0XN/0HvsU9FjTxqhFghDJi4g/yKtzcDnge1y6h91n4y7v9tQ5zif1+x73R3995cL1jefwxhtmxX1oWy/xvk9Gvd3X5I0JeKrqYeZpeEPo9ffHj0+34Hn3t7ztzH7wrGi1h+L+J5o3T8h3PRiP8JXysO+93wHvbK19cPGoLOwew5RX57+MoPGHsZnj5dnnjsueu7DhMDQr/f7lLct4aKgdwPnMRNWthACdbb+UffJJPZ30+ucVOD9KmGGjt9Gj72qYJ3DGHabFfWhbL/6F34O+22GgVeSNLJHM3Nm5zfkj0eMnc3MAeQL0ePzHXi+1Xt+I7On0Yo9ljD28QBmxuP1x+CtJv+GGMOcQZ3voFe2tngas/9esNzrhqgvTxxsitb9qug1n848tz3hTOMWwtRYR0Svzbswr8gTCeOQXwE8Lef5pzAzj/MDOfWPuk/K7u+21DnOe8bL/pqZiyRfFj1+F+Hr/2GU3WZFfSjbr29FjxfNmRv/AWfglSSVEl/kdBHFF7/EF0ltIH96p6IDz/HRa/425/kdCcGsf9DuH8z7YfwO8i+yOmaI974hek129ohxaosvGrogp75dCDeuGDfwXszsG0z0a/pJ9JoX5qwjvtCuP+3TeoYPRBBueNFfxzXMHWO5ELiTmfCXrX/UIFJ2f7elznHec9Cy/xY9d1bBerPKbrOiOsr2653R4xcyd3s+jDBG18ArSRrLIuArzBwYbgP+N2H4wgGEuWWzNz94V2Yd8x14tmP2wedfCHNlHgAcy+z5QN8cLXdt9Pg3CGezngkcRbhAKP7a9T7CPLVZ10SvOYUwo0L8urK1bc/sQPstwtmygwjjfG9m9jYrG3i3EMZaH9Nb93HMDrvXkH+h4ME56/naCDVAGJKxJlr+XOClhG3zUoqDVtkgUnZ/t6XOcd5z0LKPYmbqvC2EPs+n7DYrqqNsv3Zk9lR6/Xl4DwJe21tuLZN9T0nSlFpIGM/b/wqzqK0hnBnKGibU7c3sMzV5LTun6fMYfDerFcxc5V10YDu9YNlxa4PwFeztA5aJ570tE3hvYXbgzrZbCMMtimT79LIRauh7NbMDXF77HrOHg5QNIuPs77bUWUXghdlzA9/OcHPyltlmRXWM06+nMfsmLnHbhEMaJEkT9mhCsLuGcMvRjYSLSZYTbhFcNJPDsKFuW8JXmMuB+wkH21sJY4Pz5u+FcFvQcwhfr24gXHx1EWGIxTaEu5rdSvi6fkXO8tsBf0MYd7m+16e8adjK1AYhWJxG6PvvCGfariScDV7AeIH3RsIZsA8CPyb8QbKRMMTidOafe/XkaF33kj+kYxi/D5xJOLP8IGHb3EEYykfGXgAACGBJREFUyvE65g65GCeIjLO/21BnVYF3EeGCxP5r/l/B+rNG3WZFdYwbPncjDCm6mbDNHgS+CRzK7N+jSW1LSZLUEX/J7LPNUhvtwczn+MLEtUiSpAZZDPycmaDwlMEvl5I4nvDNxTpgacFr3sfM5/h9NdUlSZIa7C2EC9zOYSYkfDNpRVKxxzIzHnoj8FHg+YSLS/sXAfZvgf4A8IgkVUqSpEbJXvSzmpm7rUlN9EpmzxiR1+4h3BVOkiSJOwlnyh4k3Bxj37TlSEN5POHi0usIf6RtJMx0sRz4AOGiNkmSJEmSJEmSJEmSJEmSJEmSJLXINsDfEe6wtIlw7/p3JK2oOvGV3yfN89ppuFPToCvjtxDmR70Z+AfCrZRT1TfKneqq8lbCHQeXpS1DkiSVcSpzg86ylAVVKO7jGsLV4UUMvLPbJuDtieprQuCdhs+DJEksTl1ARQ6P/vsvgGuBuxLVUqdtgU8Bz0tdSAP8CnhFzuMPJcx/+m7gIcDHgG8D36+prv16P9fW9H6SJKmjbmB6zlzlnbl8TcFrp+GM3rBnUF8fvfZTVRfVUNPweZAkqXMGfX2dfc2NwALgz4GfEL7eXha9bhFwHOE2tvf1nr+v9///o/d80fvfCCwkjJFcAawn3Eb0q8CTe699JPCPhHHGGwg3WDgHeFLJPt9LmOC+fxenvFuWzhdwxu1znqL3zC73p8APCdvqXuDfgacXrHOQYQPvTtFrry5YftKfj/nqW9hb5xXASsLn4pfAmcBeA/qyFeGbjOXAf/aWuwM4F3hVrx95Ncz3uyJJkhpo1MD76cxrlvVe80jgqnnWd2XvdXnvfyPwmYLlVhKGHNw24PknlOjzjcA7o///Qs5rBwXeSfQ5zzCB94MF77eW0e+sNmzg3S567Q8Klp/052NQfQ8j3EmuaH0rgRfkrG9Pwh8Kg2o5jzB8I1uDgVeSpBbat9duYebg3X+sr//4g72fywljPQ8gXPC1mNlh5lrgWODg3s9ro+euYvY46P7jmwhn2T4MPAt4MeFMb//5zYQxxW8CDgJezuxg+MkR+hwHqEXA96LHDs+8tih8TqLPZQPvOsK2+ivg2YRQF9fy9eKu5xo28B4dvfaLOctX8fkoqm8BcD4zn43PAkcSPhsnEM7W9rdVPLPEVsze35cAr+zVehzw0+i5j0XLDfo98dbNkiS1xKAzmfGZrCuArTPPHxc9v5zZZ8YgTHm2PHrNcQXrfktmuf0zz2fD6MHRc98d1LmC/vQD1NOAjb3Hfkk4k9lXtF0m0eeygXcLc2dK2JPwR8MWwjCBUcxXz26Er/9XRq89qqCuSX8+iup7SfT48Tk1LwFW9Z6/NHr8NdFyX2fu0IVHAPczE+C3yTzvGF5Jklps2MB7cM7zF0bPP7Ng/QdGr/lGzrpXE86+xbaOnn+AueFk++j5GwreN09egPrr6PGPRo8XbZdJ9Lls4F3D3CAGYZaFMmFsy4jtHwcsP+nPR7z+eHt9jZlwXzT297PRskt6j10QPbZPwXLHAO/rtYdnnjPwSpLUYsME3pWEi4Sy7mYmlBZZQLg4aAuzpzvrr/tnBcsNen4B+WFoPnnLbAf8ovf4RmYu/iraLpPoc9nAW7StyoaxYYPuCvJns6jy8xGvP95edzB83VsIIRbChY5bCBcplmHglSRNha7OwzuMOwnjJbN26v0cFCK2EILPQ6PXx/LWO9/zkwwdawizC1xAOGP4GcKQiiKT6HNZ822rsvLm4d0C/A74DSGQDlLl5yNrlyFeE9s+s9yowz4kSZoq0xx4NxU8/gCwa68VWcDMtF+DzvSl9A3CxVivJpzhPWHAa6vsc9FX9FVbR7igq6w6Px+rCOH1duBFQ7z+F9FyO/eaJEkqkPeV7bTrT1H1MMLV7nkOJJy9i1/fRO8gzGkLcAr5c/PCeH1e3/u5Q84yi4A9hqq0Par4fFzf+7kb4ULD7+W0ewh/oC4mXIAG8OPez12BJxas+3LCsJYNFO9/SZLUQsOM4S0ac/ra6DXLmXtBVfYq/NeMsO5xny+zzGvIHwcaG6fPv4weX5JZLp76q2gM76hjf+dTZhuOsvw426po/cdHj/9tznvuyMyY7LuY+Wbm7dFyX2LuhZD7E4ZlbCF/5o/4joTZ2SgkSVLDjRN4twKuiV53LeEuYAf1fsbzrF7NaHPSpgi8EO7+NSjwjtPnf4ie+wHhK/mDgfcSZqvYTP57tjXwjrOtita/HbP7+y+E+YgPIMztG8/h/ObMcvFcu+cTtv9BhDB8b/Rc3k0r4n6cQgjIBxX0W5IkNcw4gRfC1/BxGMhr1zD36/qmBt69CXctKwq8UL7PezN7Ttu4nc3wtxbOamrghfLbatD69ybMWDFonR/JWd/ewE0DltkMnFzQj9MLlpEkSS0wbuCFcGbudcBFhCvuNxC+Tr6Q8LV23gV/TQ28ACcxf6gp02cIc8CeRZheawPhDl4nE8bwdjHwQvltNWj92xJuD72ccMOIDcCthD8c8uYE7tsOeBfhdsb3E8ZV3wqcQ7jT36Dl/oYwo8V6wljf7DRqkiRJ0kj6gXeUm4tIkqQxOEuDVJ/4NsTrklUhSdKUmeZ5eKU6HQzsF/3/TakKkSRJkqqQvTDsyLTlSJIkSZO1lnAR2g3A6xPXIkmSJEmSJEmSJEmSJEmSJEmSJEmSJEmSJEmSJEmSJEmSJEmSJEmSJEmSJEmSJEmSJEmSJEmSJEmSJEmSJEmSJEmSJEmSJEmSJEmSJEmSJEmSJEmSJEmSJEmSJEmSJEmSJEmSJEmSJEmSJEmSJEmSJEmSJEmSJEmSJEmSJEmSJEmSJEmSJEmSJEmSJEmSJEmSJEmSJEmSJEmS1Gz/H0jIHmpL+iKoAAAAAElFTkSuQmCC",
                fileName="modelica://ProsNet/../../../../Downloads/noun_Snow_1959861.png")}),
            Diagram(coordinateSystem(preserveAspectRatio=false)));
      end heat_source_sink_ideal;

      model Conversion

      package Medium = Modelica.Media.Water.ConstantPropertyLiquidWater;

        Modelica.Blocks.Sources.Constant factor1(k=(1/60000)*Medium.d_const)
          annotation (Placement(transformation(extent={{-52,40},{-32,60}})));
        Modelica.Blocks.Math.Product volume2mass_flow
          annotation (Placement(transformation(extent={{-10,22},{10,42}})));
        ProsNet.Controls.SecondaryFlowControl secFlowCon
          annotation (Placement(transformation(extent={{30,4},{50,24}})));
        ProsNet.Controls.PrimaryFlowControl priFlowCon
          annotation (Placement(transformation(extent={{-8,-26},{12,-46}})));
        ProsNet.Controls.Linearizer lin(redeclare final
            ProsNet.Controls.Data.Linearizer.EqualPercentage cha) annotation (
            Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={-30,-70})));
        Modelica.Blocks.Interfaces.RealInput T_sec_in_set "Kelvin"
          annotation (Placement(transformation(extent={{-120,120},{-80,160}})));
        Modelica.Blocks.Interfaces.RealInput V_dot_sec_set "l/min"
          annotation (Placement(transformation(extent={{-120,80},{-80,120}})));
        Modelica.Blocks.Interfaces.RealInput pi "{0;1}"
          annotation (Placement(transformation(extent={{-120,-20},{-80,20}})));
        Modelica.Blocks.Interfaces.RealInput mu "{-1;0;1}"
          annotation (Placement(transformation(extent={{-120,-62},{-80,-22}})));
        Modelica.Blocks.Interfaces.RealInput kappa_set "[0;1]"
          annotation (Placement(transformation(extent={{-120,-140},{-80,-100}})));
        Modelica.Blocks.Interfaces.RealInput u_set "[0;1]"
          annotation (Placement(transformation(extent={{-120,-100},{-80,-60}})));
        Modelica.Blocks.Interfaces.RealInput m_dot_sec_is "kg/s"
          annotation (Placement(transformation(extent={{120,100},{80,140}})));
        Modelica.Blocks.Interfaces.RealOutput m_dot_prod "kg/s"
          annotation (Placement(transformation(extent={{90,70},{110,90}})));
        Modelica.Blocks.Interfaces.RealOutput m_dot_cons "kg/s"
          annotation (Placement(transformation(extent={{90,30},{110,50}})));
        Modelica.Blocks.Interfaces.RealOutput pump_contr "[0;1]"
          annotation (Placement(transformation(extent={{90,-50},{110,-30}})));
        Modelica.Blocks.Interfaces.RealOutput valve_contr "[0;1]"
          annotation (Placement(transformation(extent={{90,-90},{110,-70}})));
        Modelica.Blocks.Interfaces.RealOutput T_sec_in annotation (Placement(
              transformation(
              extent={{-10,-10},{10,10}},
              rotation=90,
              origin={40,180})));
        Modelica.Blocks.Interfaces.RealOutput V_dot_sec_is annotation (Placement(
              transformation(
              extent={{-10,-10},{10,10}},
              rotation=90,
              origin={0,180})));
        Modelica.Blocks.Math.Product mass2volume_flow
          annotation (Placement(transformation(extent={{36,50},{16,70}})));
        Modelica.Blocks.Sources.Constant factor2(k=(60000)*(1/Medium.d_const))
          annotation (Placement(transformation(extent={{76,34},{56,54}})));
        Modelica.Blocks.Interfaces.RealInput m_dot_prim_is "kg/s"
          annotation (Placement(transformation(extent={{120,-140},{80,-100}})));
        Modelica.Blocks.Interfaces.RealOutput V_dot_prim_is annotation (Placement(
              transformation(
              extent={{-10,-10},{10,10}},
              rotation=-90,
              origin={0,-160})));
        Modelica.Blocks.Math.Product mass2volume_flow1
          annotation (Placement(transformation(extent={{48,-132},{28,-112}})));
        Modelica.Blocks.Sources.Constant factor3(k=(60000)*(1/Medium.d_const))
          annotation (Placement(transformation(extent={{82,-108},{62,-88}})));
        Modelica.Blocks.Math.RealToInteger realToInteger
          annotation (Placement(transformation(extent={{-72,-10},{-52,10}})));
        Modelica.Blocks.Math.RealToInteger realToInteger1
          annotation (Placement(transformation(extent={{-72,-52},{-52,-32}})));
      equation
        connect(factor1.y, volume2mass_flow.u1) annotation (Line(points={{-31,50},{-18,
                50},{-18,38},{-12,38}}, color={0,0,127}));
        connect(T_sec_in_set, T_sec_in)
          annotation (Line(points={{-100,140},{40,140},{40,180}}, color={0,0,127}));
        connect(volume2mass_flow.u2, V_dot_sec_set) annotation (Line(points={{-12,26},
                {-74,26},{-74,100},{-100,100}}, color={0,0,127}));
        connect(factor2.y, mass2volume_flow.u2) annotation (Line(points={{55,44},{44,44},
                {44,54},{38,54}}, color={0,0,127}));
        connect(m_dot_sec_is, mass2volume_flow.u1) annotation (Line(points={{100,120},
                {48,120},{48,66},{38,66}}, color={0,0,127}));
        connect(mass2volume_flow.y, V_dot_sec_is)
          annotation (Line(points={{15,60},{0,60},{0,180}}, color={0,0,127}));
        connect(volume2mass_flow.y, secFlowCon.m_flow_set)
          annotation (Line(points={{11,32},{40,32},{40,26}}, color={0,0,127}));
        connect(secFlowCon.m_flow_production, m_dot_prod) annotation (Line(points={{51,19.1},
                {51,32},{84,32},{84,64},{74,64},{74,80},{100,80}},       color={0,0,127}));
        connect(secFlowCon.m_flow_consumption, m_dot_cons) annotation (Line(points={{51,8.9},
                {62,8.9},{62,28},{82,28},{82,40},{100,40}},      color={0,0,127}));
        connect(priFlowCon.pump_y, pump_contr) annotation (Line(points={{13,-30.9},{84,
                -30.9},{84,-40},{100,-40}}, color={0,0,127}));
        connect(priFlowCon.valve_op, valve_contr) annotation (Line(points={{13,-41.1},
                {82,-41.1},{82,-80},{100,-80}}, color={0,0,127}));
        connect(lin.op, priFlowCon.valve_op_set)
          annotation (Line(points={{-19,-70},{6,-70},{6,-48}}, color={0,0,127}));
        connect(kappa_set, lin.kappa) annotation (Line(points={{-100,-120},{-52,-120},
                {-52,-70},{-42,-70}}, color={0,0,127}));
        connect(u_set, priFlowCon.pump_y_set) annotation (Line(points={{-100,-80},{-54,
                -80},{-54,-54},{-10,-54},{-10,-56},{-2,-56},{-2,-48}}, color={0,0,127}));
        connect(factor3.y, mass2volume_flow1.u1) annotation (Line(points={{61,-98},
                {60,-98},{60,-116},{50,-116}},
                                           color={0,0,127}));
        connect(m_dot_prim_is, mass2volume_flow1.u2) annotation (Line(points={{100,
                -120},{58,-120},{58,-128},{50,-128}},
                                                color={0,0,127}));
        connect(mass2volume_flow1.y, V_dot_prim_is)
          annotation (Line(points={{27,-122},{0,-122},{0,-160}}, color={0,0,127}));
        connect(realToInteger.y, secFlowCon.pi)
          annotation (Line(points={{-51,0},{20,0},{20,8},{28,8}}, color={255,127,0}));
        connect(realToInteger.y, priFlowCon.pi)
          annotation (Line(points={{-51,0},{-24,0},{-24,-30},{-10,-30}}, color={255,127,0}));
        connect(realToInteger.u, pi)
          annotation (Line(points={{-74,0},{-100,0}}, color={0,0,127}));
        connect(mu, realToInteger1.u)
          annotation (Line(points={{-100,-42},{-74,-42}}, color={0,0,127}));
        connect(realToInteger1.y, priFlowCon.mu)
          annotation (Line(points={{-51,-42},{-10,-42}}, color={255,127,0}));
        connect(realToInteger1.y, secFlowCon.mu) annotation (Line(points={{-51,-42},{-32,-42},
                {-32,14},{2,14},{2,20},{28,20}}, color={255,127,0}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-160},
                  {100,180}}), graphics={Rectangle(
                extent={{-100,180},{100,-160}},
                lineColor={28,108,200},
                fillColor={171,171,171},
                fillPattern=FillPattern.Solid), Text(
                extent={{-137,35},{137,-35}},
                textColor={0,0,0},
                origin={-1,1},
                rotation=90,
                textString="Conversion",
                textStyle={TextStyle.Bold})}),
                                Diagram(coordinateSystem(preserveAspectRatio=false,
                extent={{-100,-160},{100,180}})));
      end Conversion;

      model Test_Conversion
        Conversion conversion
          annotation (Placement(transformation(extent={{-6,-2},{14,32}})));
        Modelica.Blocks.Sources.RealExpression T_sec_in(y=65)
          annotation (Placement(transformation(extent={{-86,38},{-66,58}})));
        Modelica.Blocks.Sources.IntegerExpression pi(y=0)
          annotation (Placement(transformation(extent={{-86,-2},{-66,18}})));
        Modelica.Blocks.Sources.RealExpression V_dot_sec_set(y=5)
          annotation (Placement(transformation(extent={{-86,18},{-66,38}})));
        Modelica.Blocks.Sources.IntegerExpression mu(y=-1)
          annotation (Placement(transformation(extent={{-86,-24},{-66,-4}})));
        Modelica.Blocks.Sources.RealExpression u_set(y=0.5)
          annotation (Placement(transformation(extent={{-86,-48},{-66,-28}})));
        Modelica.Blocks.Sources.RealExpression kappa_set(y=0.8)
          annotation (Placement(transformation(extent={{-86,-66},{-66,-46}})));
        Modelica.Blocks.Sources.RealExpression m_dot_sec_is(y=0.2)
          annotation (Placement(transformation(extent={{76,16},{56,36}})));
        Modelica.Blocks.Sources.RealExpression m_dot_prim_is(y=0.02)
          annotation (Placement(transformation(extent={{76,-10},{56,10}})));
      equation
        connect(kappa_set.y, conversion.kappa_set) annotation (Line(points={{-65,
                -56},{-12,-56},{-12,2},{-6,2}}, color={0,0,127}));
        connect(u_set.y, conversion.u_set) annotation (Line(points={{-65,-38},{-14,
                -38},{-14,6},{-6,6}}, color={0,0,127}));
        connect(mu.y, conversion.mu) annotation (Line(points={{-65,-14},{-16,-14},{
                -16,9.8},{-6,9.8}}, color={255,127,0}));
        connect(pi.y, conversion.pi) annotation (Line(points={{-65,8},{-18,8},{-18,
                14},{-6,14}}, color={255,127,0}));
        connect(V_dot_sec_set.y, conversion.V_dot_sec_set) annotation (Line(points=
                {{-65,28},{-14,28},{-14,24},{-6,24}}, color={0,0,127}));
        connect(T_sec_in.y, conversion.T_sec_in_set) annotation (Line(points={{-65,
                48},{-12,48},{-12,28},{-6,28}}, color={0,0,127}));
        connect(conversion.m_dot_sec_is, m_dot_sec_is.y)
          annotation (Line(points={{14,26},{55,26}}, color={0,0,127}));
        connect(conversion.m_dot_prim_is, m_dot_prim_is.y)
          annotation (Line(points={{14,2},{50,2},{50,0},{55,0}}, color={0,0,127}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
              Ellipse(lineColor = {75,138,73},
                      fillColor={255,255,255},
                      fillPattern = FillPattern.Solid,
                      extent={{-100,-100},{100,100}}),
              Polygon(lineColor = {0,0,255},
                      fillColor = {75,138,73},
                      pattern = LinePattern.None,
                      fillPattern = FillPattern.Solid,
                      points={{-36,60},{64,0},{-36,-60},{-36,60}})}),  Diagram(
              coordinateSystem(preserveAspectRatio=false)));
      end Test_Conversion;

      model Test_heat_source_sink_ideal
        heat_source_sink_ideal heat_source_sink_ideal1
          annotation (Placement(transformation(extent={{-24,2},{32,48}})));
        Modelica.Fluid.Sources.MassFlowSource_T boundary(
          use_m_flow_in=true,
          use_T_in=true,
          redeclare package Medium =
              ProsNet.Media.Water,
          nPorts=1) annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=90,
              origin={-20,-70})));
        Modelica.Fluid.Vessels.OpenTank tank(
          height=5,
          crossArea=10,
          use_portsData=false,
          redeclare package Medium =
              ProsNet.Media.Water,
          nPorts=1) annotation (Placement(transformation(extent={{30,-76},{70,-34}})));
        Modelica.Blocks.Sources.RealExpression temperature_sec(y=273.15 + 70)
          "Kelvin"
          annotation (Placement(transformation(extent={{-68,68},{-48,88}})));
        Modelica.Blocks.Sources.RealExpression temperature_in(y=273.15 + 30)
          "Kelvin"
          annotation (Placement(transformation(extent={{-96,-88},{-76,-68}})));
        Modelica.Blocks.Sources.RealExpression m_flow(y=5/60) "kg/s"
          annotation (Placement(transformation(extent={{-96,-70},{-76,-50}})));
        inner Modelica.Fluid.System system
          annotation (Placement(transformation(extent={{56,62},{76,82}})));
      equation
        connect(boundary.m_flow_in, m_flow.y) annotation (Line(points={{-28,-80},{
                -44,-80},{-44,-60},{-75,-60}}, color={0,0,127}));
        connect(boundary.T_in, temperature_in.y) annotation (Line(points={{-24,-82},
                {-24,-88},{-70,-88},{-70,-78},{-75,-78}}, color={0,0,127}));
        connect(temperature_sec.y, heat_source_sink_ideal1.T_set)
          annotation (Line(points={{-47,78},{4,78},{4,48}}, color={0,0,127}));
        connect(heat_source_sink_ideal1.port_cold, boundary.ports[1]) annotation (
            Line(points={{20.8,2},{20.8,-54},{-20,-54},{-20,-60}}, color={0,127,255}));
        connect(heat_source_sink_ideal1.port_hot, tank.ports[1]) annotation (Line(
              points={{-12.8,2},{-12.8,-56},{24,-56},{24,-84},{50,-84},{50,-76}},
              color={0,127,255}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
              Ellipse(lineColor = {75,138,73},
                      fillColor={255,255,255},
                      fillPattern = FillPattern.Solid,
                      extent={{-100,-100},{100,100}}),
              Polygon(lineColor = {0,0,255},
                      fillColor = {75,138,73},
                      pattern = LinePattern.None,
                      fillPattern = FillPattern.Solid,
                      points={{-36,60},{64,0},{-36,-60},{-36,60}})}),  Diagram(
              coordinateSystem(preserveAspectRatio=false)));
      end Test_heat_source_sink_ideal;

      model Test_heat_transfer_station_production
        Modelica.Blocks.Sources.RealExpression T_house(y=273.15 + 70)
          annotation (Placement(transformation(extent={{-92,74},{-72,94}})));
        Modelica.Blocks.Sources.IntegerExpression pi(y=1)
          annotation (Placement(transformation(extent={{-92,44},{-72,64}})));
        Modelica.Blocks.Sources.IntegerExpression mu(y=1)
          annotation (Placement(transformation(extent={{-92,30},{-72,50}})));
        Modelica.Blocks.Sources.RealExpression u_pump(y=0.5)
          annotation (Placement(transformation(extent={{-92,12},{-72,32}})));
        Modelica.Blocks.Sources.RealExpression kappa(y=0.8)
          annotation (Placement(transformation(extent={{-92,-2},{-72,18}})));
        Modelica.Blocks.Sources.RealExpression flow_house(y=5)
          annotation (Placement(transformation(extent={{-92,62},{-72,82}})));
        Modelica.Blocks.Sources.Ramp ramp(
          height=1,
          duration=600,
          offset=0,
          startTime=200)
          annotation (Placement(transformation(extent={{-92,-32},{-72,-12}})));
        heat_transfer_station heat_transfer_station1(redeclare
            ProsNet.Fluid.Pumps.Data.Pumps.QuadraticCharacteristic
                                                           feedinPer,
          energyDynamics_feedPump=Modelica.Fluid.Types.Dynamics.DynamicFreeInitial,
          use_inputFilter_feedPump=true,
          init_feedPump=Modelica.Blocks.Types.Init.InitialOutput,
          use_inputFilter_conVal=true,
          init_conVal=Modelica.Blocks.Types.Init.InitialOutput,
            ambient_temperature=system.T_ambient,
          energyDynamics_pumpsSec=Modelica.Fluid.Types.Dynamics.DynamicFreeInitial,
          use_inputFilter_pumpsSec=true)
          annotation (Placement(transformation(extent={{-8,40},{22,76}})));

        Modelica.Fluid.Vessels.ClosedVolume volume(
          T_start=318.15,
          use_portsData=false,                     V=1, nPorts=2,
        redeclare final package Medium = ProsNet.Media.Water)
          annotation (Placement(transformation(extent={{2,-24},{22,-44}})));
        inner Modelica.Fluid.System system(T_ambient=285.15)
          annotation (Placement(transformation(extent={{60,62},{80,82}})));
        ProsNet.Fluid.Valves.TwoWayEqualPercentage valve_for_test(
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
              origin={36,-4})));
        ProsNet.Fluid.Sources.Boundary_pT bou(nPorts=1, redeclare final package
            Medium = ProsNet.Media.Water)
          annotation (Placement(transformation(extent={{90,12},{70,32}})));
      equation
        connect(T_house.y, heat_transfer_station1.T_sec_in_set) annotation (Line(
              points={{-71,84},{-14,84},{-14,72},{-8,72}}, color={0,0,127}));
        connect(flow_house.y, heat_transfer_station1.V_dot_sec_set) annotation (Line(
              points={{-71,72},{-16,72},{-16,68},{-8,68}}, color={0,0,127}));
        connect(pi.y, heat_transfer_station1.pi) annotation (Line(points={{-71,54},{-16,
                54},{-16,62},{-8,62}}, color={255,127,0}));
        connect(mu.y, heat_transfer_station1.mu) annotation (Line(points={{-71,40},{-14,
                40},{-14,58},{-8,58}}, color={255,127,0}));
        connect(u_pump.y, heat_transfer_station1.u_set) annotation (Line(points={{-71,
                22},{-66,22},{-66,52},{-60,52},{-60,54},{-8,54}}, color={0,0,127}));
        connect(kappa.y, heat_transfer_station1.kappa_set) annotation (Line(points={{-71,
                8},{-58,8},{-58,50},{-8,50}}, color={0,0,127}));
        connect(heat_transfer_station1.hot_prim, volume.ports[1])
          annotation (Line(points={{-3.5,39.8},{-3.5,-24},{10,-24}},
                                                               color={0,127,255}));
        connect(valve_for_test.port_b, heat_transfer_station1.cold_prim) annotation (
            Line(points={{36,6},{36,34},{17.5,34},{17.5,40}},
                                                            color={0,127,255}));
        connect(valve_for_test.port_a, volume.ports[2])
          annotation (Line(points={{36,-14},{36,-24},{14,-24}}, color={0,127,255}));
        connect(ramp.y, valve_for_test.y) annotation (Line(points={{-71,-22},{0,-22},{
                0,-4},{24,-4}}, color={0,0,127}));
        connect(bou.ports[1], valve_for_test.port_b)
          annotation (Line(points={{70,22},{36,22},{36,6}}, color={0,127,255}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
              Ellipse(lineColor = {75,138,73},
                      fillColor={255,255,255},
                      fillPattern = FillPattern.Solid,
                      extent={{-100,-100},{100,100}}),
              Polygon(lineColor = {0,0,255},
                      fillColor = {75,138,73},
                      pattern = LinePattern.None,
                      fillPattern = FillPattern.Solid,
                      points={{-36,60},{64,0},{-36,-60},{-36,60}})}),  Diagram(
              coordinateSystem(preserveAspectRatio=false)),
          experiment(
            StopTime=1000,
            Interval=1,
            __Dymola_Algorithm="Dassl"));
      end Test_heat_transfer_station_production;

      model Test_heat_transfer_station_consumption
        Modelica.Blocks.Sources.RealExpression T_house(y=273.15 + 45)
          annotation (Placement(transformation(extent={{-92,74},{-72,94}})));
        Modelica.Blocks.Sources.IntegerExpression pi(y=1)
          annotation (Placement(transformation(extent={{-92,44},{-72,64}})));
        Modelica.Blocks.Sources.IntegerExpression mu(y=-1)
          annotation (Placement(transformation(extent={{-92,30},{-72,50}})));
        Modelica.Blocks.Sources.RealExpression u_pump(y=0.5)
          annotation (Placement(transformation(extent={{-92,12},{-72,32}})));
        Modelica.Blocks.Sources.RealExpression kappa(y=0.8)
          annotation (Placement(transformation(extent={{-92,-2},{-72,18}})));
        Modelica.Blocks.Sources.RealExpression flow_house(y=5)
          annotation (Placement(transformation(extent={{-92,62},{-72,82}})));
        Modelica.Blocks.Sources.Ramp ramp(
          height=1,
          duration=600,
          offset=0,
          startTime=200)
          annotation (Placement(transformation(extent={{-92,-32},{-72,-12}})));
        heat_transfer_station heat_transfer_station1(redeclare
            ProsNet.Fluid.Pumps.Data.Pumps.QuadraticCharacteristic
                                                           feedinPer,
          energyDynamics_feedPump=Modelica.Fluid.Types.Dynamics.DynamicFreeInitial,
          use_inputFilter_feedPump=true,
          init_feedPump=Modelica.Blocks.Types.Init.InitialOutput,
          use_inputFilter_conVal=true,
          init_conVal=Modelica.Blocks.Types.Init.InitialOutput,
            ambient_temperature=system.T_ambient,
          energyDynamics_pumpsSec=Modelica.Fluid.Types.Dynamics.DynamicFreeInitial,
          use_inputFilter_pumpsSec=true,
          energyDynamics_cv=Modelica.Fluid.Types.Dynamics.SteadyState)
          annotation (Placement(transformation(extent={{-8,40},{22,76}})));

        Modelica.Fluid.Vessels.ClosedVolume volume(
          T_start=338.15,
          use_portsData=false,                     V=1, nPorts=2,
        redeclare final package Medium = ProsNet.Media.Water)
          annotation (Placement(transformation(extent={{2,-30},{22,-50}})));
        inner Modelica.Fluid.System system(T_ambient=285.15)
          annotation (Placement(transformation(extent={{62,62},{82,82}})));
        ProsNet.Fluid.Sources.Boundary_pT bou(redeclare final package Medium =
              ProsNet.Media.Water, nPorts=1)
          annotation (Placement(transformation(extent={{90,12},{70,32}})));
        ProsNet.Fluid.Pumps.SpeedControlled_y pump_prim_prod(
          redeclare final package Medium = ProsNet.Media.Water,
          final energyDynamics=Modelica.Fluid.Types.Dynamics.DynamicFreeInitial,
          final tau=1,
          redeclare final ProsNet.Fluid.Pumps.Data.Pumps.IMP.NMTPlus152025to40
            per,
          final use_inputFilter=true,
          final riseTime=5,
          final init=Modelica.Blocks.Types.Init.InitialOutput,
          final y_start=0) annotation (Placement(transformation(
              extent={{10,10},{-10,-10}},
              rotation=-90,
              origin={40,2})));
      equation
        connect(T_house.y, heat_transfer_station1.T_sec_in_set) annotation (Line(
              points={{-71,84},{-14,84},{-14,72},{-8,72}}, color={0,0,127}));
        connect(pi.y, heat_transfer_station1.pi) annotation (Line(points={{-71,54},{-16,
                54},{-16,62},{-8,62}}, color={255,127,0}));
        connect(mu.y, heat_transfer_station1.mu) annotation (Line(points={{-71,40},{-14,
                40},{-14,58},{-8,58}}, color={255,127,0}));
        connect(u_pump.y, heat_transfer_station1.u_set) annotation (Line(points={{-71,
                22},{-66,22},{-66,52},{-60,52},{-60,54},{-8,54}}, color={0,0,127}));
        connect(kappa.y, heat_transfer_station1.kappa_set) annotation (Line(points={{-71,
                8},{-58,8},{-58,50},{-8,50}}, color={0,0,127}));
        connect(heat_transfer_station1.hot_prim, volume.ports[1])
          annotation (Line(points={{6,39.8},{6,-30},{10,-30}}, color={0,127,255}));
        connect(volume.ports[2], pump_prim_prod.port_a) annotation (Line(points={{14,-30},
                {22,-30},{22,-16},{40,-16},{40,-8}}, color={0,127,255}));
        connect(heat_transfer_station1.cold_prim, pump_prim_prod.port_b) annotation (
            Line(points={{16,39.8},{16,18},{40,18},{40,12}}, color={0,127,255}));
        connect(ramp.y, pump_prim_prod.y) annotation (Line(points={{-71,-22},{10,-22},
                {10,2},{28,2}}, color={0,0,127}));
        connect(bou.ports[1], pump_prim_prod.port_b)
          annotation (Line(points={{70,22},{40,22},{40,12}}, color={0,127,255}));
        connect(flow_house.y, heat_transfer_station1.V_dot_sec_set) annotation (
            Line(points={{-71,72},{-16,72},{-16,68},{-8,68}}, color={0,0,127}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
              Ellipse(lineColor = {75,138,73},
                      fillColor={255,255,255},
                      fillPattern = FillPattern.Solid,
                      extent={{-100,-100},{100,100}}),
              Polygon(lineColor = {0,0,255},
                      fillColor = {75,138,73},
                      pattern = LinePattern.None,
                      fillPattern = FillPattern.Solid,
                      points={{-36,60},{64,0},{-36,-60},{-36,60}})}),  Diagram(
              coordinateSystem(preserveAspectRatio=false)),
          experiment(
            StopTime=1000,
            Interval=1,
            __Dymola_Algorithm="Dassl"));
      end Test_heat_transfer_station_consumption;
      annotation (conversion(noneFromVersion=""));
    end new_prosumer_models;

    package Controller_PID_based

      model PID_Q_T_weighted_sameside

        import Modelica.Units.SI;
        import T_AbsZeroDegC = Modelica.Constants.T_zero;
        import Modelica.Blocks.Types.Init;
        import Modelica.Blocks.Types.SimpleController;

          // !!!!! parameters !!!!!
        parameter Real Delta_Qdot_norm = 1
            "Heat power value for normalizing the error (deviation) of the transferred heat.
      For alpha=0.5 a deviation of Delta_Qdot_norm in heat transfer is weigthed equal to a deviation of Delta_T_norm in temperature."
            annotation(Dialog(group="Normalizing values"));
        parameter SI.TemperatureDifference Delta_T_norm(min=0) = 3
            "Temperature difference for normalizing the error (deviation) of the temperature.
      For alpha=0.5 a deviation of Delta_T_norm in temperature is weighted equal to a deviation of Delta_Qdot_norm in heat transfer."
            annotation(Dialog(group="Normalizing values"));

        parameter SI.Temperature T_prim_hot_des(min=277)= - T_AbsZeroDegC + 53.5
            "desired temperature supply primary side"
            annotation(Dialog(group="Temperature objectives"));
        parameter SI.Temperature T_sec_hot_des(min=277)= - T_AbsZeroDegC + 50
            "desired temperature supply secondary side"
            annotation(Dialog(group="Temperature objectives"));
        parameter SI.TemperatureDifference DeltaT_prim_des(min=1) =   20
            "desired temperature difference primary side"
            annotation(Dialog(group="Temperature objectives"));
        parameter SI.TemperatureDifference DeltaT_sec_des(min=1) =   20
            "desired temperature difference secondary side"
            annotation(Dialog(group="Temperature objectives"));
        parameter Real V_dot_sec_max(unit="l/min", displayUnit="l/min") = 8.5
          "maximum secondary side volume flow in [l/min]"
          annotation(Dialog(group="General PID settings"));
        parameter Real k_prim_prod = 1.5
          "Proportional gain for controller in [-]"
          annotation(Dialog(group="PID primary side - producer mode - tuning"));
        parameter Real Ti_prim_prod = 8
          "Integral time constant for controller in [s]"
          annotation(Dialog(group="PID primary side - producer mode - tuning"));
        parameter Real Td_prim_prod = 0
          "Derivative time constant for controller in [s]"
          annotation(Dialog(group="PID primary side - producer mode - tuning"));
        parameter Real alpha_prim_prod(min=0, max=1) = 0.333
          "weight for the relevance of the error of the transferred heat in comparison to the error of temperature objectives (sum is one)"
          annotation(Dialog(group="PID primary side - producer mode - tuning"));
        parameter Real k_sec_prod = 1.5
          "Proportional gain for controller in [-]"
          annotation(Dialog(group="PID secondary side - producer mode - tuning"));
        parameter Real Ti_sec_prod = 8
          "Integral time constant for controller in [s]"
          annotation(Dialog(group="PID secondary side - producer mode - tuning"));
        parameter Real Td_sec_prod = 0
          "Derivative time constant for controller in [s]"
          annotation(Dialog(group="PID secondary side - producer mode - tuning"));
        parameter Real alpha_sec_prod(min=0, max=1) = 0.666
          "weight for the relevance of the error of the transferred heat in comparison to the error of temperature objectives (sum is one)"
          annotation(Dialog(group="PID secondary side - producer mode - tuning"));
        parameter Real k_prim_cons = 1.0
          "Proportional gain for controller in [-]"
          annotation(Dialog(group="PID primary side - consumer mode - tuning"));
        parameter Real Ti_prim_cons = 35
          "Integral time constant for controller in [s]"
          annotation(Dialog(group="PID primary side - consumer mode - tuning"));
        parameter Real Td_prim_cons = 0
          "Derivative time constant for controller in [s]"
          annotation(Dialog(group="PID primary side - consumer mode - tuning"));
        parameter Real alpha_prim_cons(min=0, max=1) = 0.666
          "weight for the relevance of the error of the transferred heat in comparison to the error of temperature objectives (sum is one)"
          annotation(Dialog(group="PID primary side - consumer mode - tuning"));
        parameter Real k_sec_cons = 1.5
          "Proportional gain for controller in [-]"
          annotation(Dialog(group="PID secondary side - consumer mode - tuning"));
        parameter Real Ti_sec_cons = 8
          "Integral time constant for controller in [s]"
          annotation(Dialog(group="PID secondary side - consumer mode - tuning"));
        parameter Real Td_sec_cons = 0
          "Derivative time constant for controller in [s]"
          annotation(Dialog(group="PID secondary side - consumer mode - tuning"));
        parameter Real alpha_sec_cons(min=0, max=1) = 0.333
          "weight for the relevance of the error of the transferred heat in comparison to the error of temperature objectives (sum is one)"
          annotation(Dialog(group="PID secondary side - consumer mode - tuning"));
        parameter .Modelica.Blocks.Types.SimpleController controllerType=
               Modelica.Blocks.Types.SimpleController.PID "Type of controller"
               annotation(Dialog(tab="Advanced", group="PIDs general"));
        parameter Init initType = Modelica.Blocks.Types.Init.NoInit
          "Type of initialization (1: no init, 2: steady state, 3: initial state, 4: initial output)"
          annotation(Dialog(tab="Advanced", group="PIDs general"));
        parameter Real tol = 0.1
          "tolerance [kW] for idle mode concerning heat transfer setpoint dotQ"
          annotation(Dialog(tab="Advanced", group="Miscellaneous"));

        // !!!!! variables !!!!!
        Real beta_prim_prod(min=0, max=1)
          "weight for the relevance of the error of the temperature(difference)";
        Real beta_sec_prod(min=0, max=1)
          "weight for the relevance of the error of the temperature(difference)";
        Real beta_prim_cons(min=0, max=1)
          "weight for the relevance of the error of the temperature(difference)";
        Real beta_sec_cons(min=0, max=1)
          "weight for the relevance of the error of the temperature(difference)";
        Integer  prosumer_mode
          "prosumer mode {-1;0;1}";
        Real T_prim_relev_des
            "desired value of relevant temperature (difference)
      for control of primary side";
        Real T_prim_relev_is
            "current value of relevant temperature (difference)
      for control of primary side";
        Real T_sec_relev_des
            "desired value of relevant temperature (difference)
      for control of secondary side";
        Real T_sec_relev_is
            "current value of relevant temperature (difference)
      for control of secondary side";

        Real PIDin_prim_cons_is_weighted
            "weighted input of is-values for PID_prim_cons";
        Real PIDin_prim_cons_des_weighted
            "weighted input of desired values for PID_prim_cons";
        Real PIDin_prim_prod_is_weighted
            "weighted input of is-values for PID_prim_prod";
        Real PIDin_prim_prod_des_weighted
            "weighted input of desired values for PID_prim_prod";
        Real PIDin_sec_cons_is_weighted
            "weighted input of is-values for PID_sec_cons";
        Real PIDin_sec_cons_des_weighted
            "weighted input of desired values for PID_sec_cons";
        Real PIDin_sec_prod_is_weighted
            "weighted input of is-values for PID_sec_prod";
        Real PIDin_sec_prod_des_weighted
            "weighted input of desired values for PID_sec_prod";

        Real error_prim_weighted
            "weighted overall error of primary side controller";

        Real error_sec_weighted
            "weighted overall error of primary side controller";

        Real error_T_prim_abs
            "temperature error of primary side controller";

        Real error_T_sec_abs
            "temperature error of primary side controller";

        Real error_Q_abs
            "temperature error of primary side controller";

        Real error_T_high_prio_abs
            "error of higher prioritized temperature objective";

        Real error_T_low_prio_abs
            "error of lower prioritized temperature objective";

        Real Delta_T_prim
            "weighted overall error of primary side controller";

        Real Delta_T_sec
            "weighted overall error of primary side controller";

        Real Q_dot_is_use;
        Real Q_dot_set_use;

         Real Delta_p_prim(unit="Pa", displayUnit="bar") annotation (
            Placement(transformation(
              extent={{-20,-20},{20,20}},
              rotation=180,
              origin={-80,-140})));

        // !!!!! ports !!!!!

        Modelica.Blocks.Interfaces.RealVectorInput states[8]
          annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));

        Modelica.Blocks.Interfaces.RealVectorOutput contr_vars_real[6]
          annotation (Placement(transformation(extent={{100,-20},{140,20}})));

        Real u_set
          "Normalized velocity of feed-in pump"
            annotation (Placement(transformation(
              extent={{-20,-20},{20,20}},
              rotation=0,
              origin={80,-60}),  iconTransformation(
              extent={{-20,-20},{20,20}},
              rotation=0,
              origin={100,-60})));
        Real kappa_set
          "Normalized flow coefficient for control valve"
           annotation (Placement(transformation(
              extent={{-20,-20},{20,20}},
              rotation=0,
              origin={80,-100}),iconTransformation(
              extent={{-20,-20},{20,20}},
              rotation=0,
              origin={100,-100})));
        Real pi_set
          "Participation" annotation (Placement(transformation(
              extent={{-20,-20},{20,20}},
              rotation=0,
              origin={80,20}),    iconTransformation(
              extent={{-20,-20},{20,20}},
              rotation=0,
              origin={100,20})));
        Real mu_set
          "Operating mode" annotation (Placement(transformation(
              extent={{-20,-20},{20,20}},
              rotation=0,
              origin={80,-20}),  iconTransformation(
              extent={{-20,-20},{20,20}},
              rotation=0,
              origin={100,-20})));
        Real T_sec_hot(
          unit="K",
          displayUnit="degC",
          min=277) "current temperature hot level secondary side"      annotation (Placement(
              transformation(
              extent={{-20,-20},{20,20}},
              rotation=0,
              origin={-80,60})));
        Real T_sec_cold(
          unit="K",
          displayUnit="degC",
          min=277)
                 "current temperature cold  level secondary side"    annotation (Placement(
              transformation(
              extent={{-20,-20},{20,20}},
              rotation=0,
              origin={-80,20})));
        Real T_prim_hot(
          unit="K",
          displayUnit="degC",
          min=277)
                 "current temperature hot level primary side"        annotation (Placement(
              transformation(
              extent={{-20,-20},{20,20}},
              rotation=0,
              origin={-80,140})));
        Real T_prim_cold(
          unit="K",
          displayUnit="degC",
          min=277)
                  "current temperature cold level primary side"       annotation (Placement(
              transformation(
              extent={{-20,-20},{20,20}},
              rotation=0,
              origin={-80,100})));
        Modelica.Blocks.Interfaces.RealInput Q_dot_set(unit="kW", displayUnit="kW")
          "setpoint heat transfer (positive production, negative consumption)"
          annotation (Placement(transformation(extent={{-20,-20},{20,20}},
              rotation=-90,
              origin={-60,188})));
        Real V_dot_prim(unit="l/min", displayUnit="l/min")
          annotation (Placement(transformation(extent={{-100,-40},{-60,0}})));
        Real V_dot_sec(unit="l/min", displayUnit="l/min")
          annotation (Placement(transformation(extent={{-100,-80},{-60,-40}})));

        Real Q_dot_is(unit="kW", displayUnit="kW")
          "currently transferred heat (positive production, negative consumption)"
          annotation (Placement(transformation(extent={{-100,-120},{-60,-80}})));
        Modelica.Blocks.Continuous.LimPID PID_prim_cons(
          controllerType=controllerType,
          k=k_prim_cons,
          Ti=Ti_prim_cons,
          Td=Td_prim_cons,
          yMax=1,
          yMin=0.25,
          initType=initType,
          y_start=PID_prim_cons.yMax)
          annotation (Placement(transformation(extent={{-30,-32},{-10,-12}})));
        Modelica.Blocks.Continuous.LimPID PID_sec_cons(
          controllerType=controllerType,
          k=k_sec_cons,
          Ti=Ti_sec_cons,
          Td=Td_sec_cons,
          yMax=V_dot_sec_max,
          yMin=2,
          initType=initType,
          y_start=PID_sec_cons.yMax)
          annotation (Placement(transformation(extent={{-34,28},{-14,48}})));
        Modelica.Blocks.Continuous.LimPID PID_prim_prod(
          controllerType=controllerType,
          k=k_prim_prod,
          Ti=Ti_prim_prod,
          Td=Td_prim_prod,
          yMax=1,
          yMin=0.25,
          initType=initType,
          y_start=PID_prim_prod.yMax) annotation (Placement(transformation(extent={{12,-30},
                  {32,-10}})));
        Modelica.Blocks.Continuous.LimPID PID_sec_prod(
          controllerType=controllerType,
          k=k_sec_prod,
          Ti=Ti_sec_prod,
          Td=Td_sec_prod,
          yMax=V_dot_sec_max,
          yMin=2,
          initType=initType,
          y_start=PID_sec_prod.yMax) annotation (Placement(transformation(extent={{10,28},
                  {30,48}})));
        Real T_sec_set(unit="K", displayUnit="degC")
           "Temperature on the secondary side" annotation (Placement(transformation(
              extent={{-20,-20},{20,20}},
              rotation=0,
              origin={80,100})));
        Real V_dot_sec_set(unit="l/min", displayUnit=
             "l/min") "volume flow rate setpoint on the secondary side" annotation (Placement(transformation(
              extent={{-20,-20},{20,20}},
              rotation=0,
              origin={80,60})));
        Modelica.Blocks.Interfaces.RealInput T_sec_in_is(unit="K", displayUnit="degC")
          "Temperature on the secondary side" annotation (Placement(transformation(
              extent={{-20,-20},{20,20}},
              rotation=270,
              origin={60,190})));

      equation

        // assign inputs
        T_prim_hot   = states[1];
        T_prim_cold  = states[2];
        T_sec_hot    = states[3];
        T_sec_cold   = states[4];
        V_dot_prim   = states[5];
        V_dot_sec    = states[6];
        Q_dot_is     = states[7];
        Delta_p_prim = states[8];

        Delta_T_prim      = T_prim_hot -T_prim_cold;
        Delta_T_sec       = T_sec_hot  -T_sec_cold;

        beta_prim_prod = 1 - alpha_prim_prod;
        beta_sec_prod  = 1 - alpha_sec_prod;
        beta_prim_cons = 1 - alpha_prim_cons;
        beta_sec_cons  = 1 - alpha_sec_cons;

        // determine easy static values that just depend on prosumer mode
        // determine inputs for the four PIDs
        // four PIDs in order to be able to have different gains for each situation

        if  Q_dot_set <= 0-tol then // consumption mode
          prosumer_mode = -1;
        elseif Q_dot_set >= 0+tol then // production mode
          prosumer_mode = +1;
        else // idle mode
          prosumer_mode = 0;
        end if;

        Q_dot_is_use = abs(Q_dot_is);
        Q_dot_set_use = abs(Q_dot_set);

        if prosumer_mode == -1 then // consumption mode
          pi_set = 1;
          mu_set = -1;
          T_prim_relev_des = DeltaT_prim_des;
          T_prim_relev_is = T_prim_hot-T_prim_cold;
          T_sec_relev_des = T_sec_hot_des;
          T_sec_relev_is = T_sec_hot;

          PIDin_prim_cons_is_weighted    = alpha_prim_cons*Q_dot_is_use/Delta_Qdot_norm + beta_prim_cons*(-1)*T_prim_relev_is/Delta_T_norm;
          PIDin_prim_cons_des_weighted   = alpha_prim_cons*Q_dot_set_use/Delta_Qdot_norm + beta_prim_cons*(-1)*T_prim_relev_des/Delta_T_norm;
          PIDin_prim_prod_is_weighted    = 0;
          PIDin_prim_prod_des_weighted   = 0;
          PIDin_sec_cons_is_weighted     = alpha_sec_cons*Q_dot_is_use/Delta_Qdot_norm + beta_sec_cons*(-1)*T_sec_relev_is/Delta_T_norm;
          PIDin_sec_cons_des_weighted    = alpha_sec_cons*Q_dot_set_use/Delta_Qdot_norm + beta_sec_cons*(-1)*T_sec_relev_des/Delta_T_norm;
          PIDin_sec_prod_is_weighted     = 0;
          PIDin_sec_prod_des_weighted    = 0;

          error_prim_weighted            = PIDin_prim_cons_des_weighted - PIDin_prim_cons_is_weighted;
          error_sec_weighted             = PIDin_sec_cons_des_weighted - PIDin_sec_cons_is_weighted;

          error_T_prim_abs               = DeltaT_prim_des - Delta_T_prim;
          error_T_sec_abs                = T_sec_hot_des - T_sec_hot;

          error_T_high_prio_abs          = T_sec_hot_des - T_sec_hot;
          error_T_low_prio_abs           = DeltaT_prim_des - Delta_T_prim;

        elseif prosumer_mode == 1 then // production mode
          pi_set = 1;
          mu_set = 1;
          T_prim_relev_des = T_prim_hot_des;
          T_prim_relev_is = T_prim_hot;
          T_sec_relev_des = DeltaT_sec_des;
          T_sec_relev_is = T_sec_hot-T_sec_cold;

          PIDin_prim_cons_is_weighted    = 0;
          PIDin_prim_cons_des_weighted   = 0;
          PIDin_prim_prod_is_weighted    = alpha_prim_prod*Q_dot_is_use/Delta_Qdot_norm + beta_prim_prod*(-1)*T_prim_relev_is/Delta_T_norm;
          PIDin_prim_prod_des_weighted   = alpha_prim_prod*Q_dot_set_use/Delta_Qdot_norm + beta_prim_prod*(-1)*T_prim_relev_des/Delta_T_norm;
          PIDin_sec_cons_is_weighted     = 0;
          PIDin_sec_cons_des_weighted    = 0;
          PIDin_sec_prod_is_weighted     = alpha_sec_prod*Q_dot_is_use/Delta_Qdot_norm + beta_sec_prod*(-1)*T_sec_relev_is/Delta_T_norm;
          PIDin_sec_prod_des_weighted    = alpha_sec_prod*Q_dot_set_use/Delta_Qdot_norm + beta_sec_prod*(-1)*T_sec_relev_des/Delta_T_norm;

          error_prim_weighted            = PIDin_prim_prod_des_weighted - PIDin_prim_prod_is_weighted;
          error_sec_weighted             = PIDin_sec_prod_des_weighted - PIDin_sec_prod_is_weighted;

          error_T_prim_abs               = T_prim_hot_des - T_prim_hot;
          error_T_sec_abs                = DeltaT_sec_des - Delta_T_sec;

          error_T_high_prio_abs          = T_prim_hot_des - T_prim_hot;
          error_T_low_prio_abs           = DeltaT_sec_des - Delta_T_sec;

        else // idle mode
          pi_set = 0;
          mu_set = -1;
          T_prim_relev_des = 0;
          T_prim_relev_is = 0;
          T_sec_relev_des = 0;
          T_sec_relev_is = 0;

          PIDin_prim_cons_is_weighted    = 0;
          PIDin_prim_cons_des_weighted   = 0;
          PIDin_prim_prod_is_weighted    = 0;
          PIDin_prim_prod_des_weighted   = 0;
          PIDin_sec_cons_is_weighted     = 0;
          PIDin_sec_cons_des_weighted    = 0;
          PIDin_sec_prod_is_weighted     = 0;
          PIDin_sec_prod_des_weighted    = 0;

          error_prim_weighted            = 0;
          error_sec_weighted             = 0;

          error_T_prim_abs               = 0;
          error_T_sec_abs                = 0;

          error_T_high_prio_abs          = 0;
          error_T_low_prio_abs           = 0;

        end if;

        // assign PID controller inputs
        PID_prim_cons.u_s    = PIDin_prim_cons_des_weighted;
        PID_prim_cons.u_m    = PIDin_prim_cons_is_weighted;
        PID_prim_prod.u_s    = PIDin_prim_prod_des_weighted;
        PID_prim_prod.u_m    = PIDin_prim_prod_is_weighted;
        PID_sec_cons.u_s     = PIDin_sec_cons_des_weighted;
        PID_sec_cons.u_m     = PIDin_sec_cons_is_weighted;
        PID_sec_prod.u_s     = PIDin_sec_prod_des_weighted;
        PID_sec_prod.u_m     = PIDin_sec_prod_is_weighted;

        // connect secondary side temperature setpoint
        T_sec_set    =  T_sec_in_is;

        // assign PID outputs to controller outputs
        if prosumer_mode == -1 then // consumption mode
          u_set = 0;
          kappa_set =PID_prim_cons.y;
          V_dot_sec_set = PID_sec_cons.y;
        elseif prosumer_mode == 1 then // production mode
          u_set =PID_prim_prod.y;
          kappa_set = 0;
          V_dot_sec_set = PID_sec_prod.y;
        else // idle mode
          V_dot_sec_set = 0;
          u_set = 0;
          kappa_set = 0;
        end if;

        error_Q_abs = Q_dot_set_use - Q_dot_is_use;

        // assign control variables vector
        contr_vars_real[1]   =  T_sec_set;
        contr_vars_real[2]   =  V_dot_sec_set;
        contr_vars_real[3]   =  pi_set;
        contr_vars_real[4]   =  mu_set;
        contr_vars_real[5]   =  u_set;
        contr_vars_real[6]   =  kappa_set;

          annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-120,-160},{
                  120,180}}),                                           graphics={
                Text(
                extent={{-70,56},{64,-56}},
                textColor={28,108,200},
                textString="weighted
PID"),       Rectangle(extent={{-120,180},{120,-160}}, lineColor={0,0,0})}),
                                                                         Diagram(
              coordinateSystem(preserveAspectRatio=false, extent={{-120,-160},{120,180}})));
      end PID_Q_T_weighted_sameside;

      model PID_Q_T_weighted_crossover

        import Modelica.Units.SI;
        import T_AbsZeroDegC = Modelica.Constants.T_zero;
        import Modelica.Blocks.Types.Init;
        import Modelica.Blocks.Types.SimpleController;

          // !!!!! parameters !!!!!
        parameter Real Delta_Qdot_norm = 1
            "Heat power value for normalizing the error (deviation) of the transferred heat.
      For alpha=0.5 a deviation of Delta_Qdot_norm in heat transfer is weigthed equal to a deviation of Delta_T_norm in temperature."
            annotation(Dialog(group="Normalizing values"));
        parameter SI.TemperatureDifference Delta_T_norm(min=0) = 3
            "Temperature difference for normalizing the error (deviation) of the temperature.
      For alpha=0.5 a deviation of Delta_T_norm in temperature is weighted equal to a deviation of Delta_Qdot_norm in heat transfer."
            annotation(Dialog(group="Normalizing values"));

        parameter SI.Temperature T_prim_hot_des(min=277)= - T_AbsZeroDegC + 53.5
            "desired temperature supply primary side"
            annotation(Dialog(group="Temperature objectives"));
        parameter SI.Temperature T_sec_hot_des(min=277)= - T_AbsZeroDegC + 50
            "desired temperature supply secondary side"
            annotation(Dialog(group="Temperature objectives"));
        parameter SI.TemperatureDifference DeltaT_prim_des(min=1) =   20
            "desired temperature difference primary side"
            annotation(Dialog(group="Temperature objectives"));
        parameter SI.TemperatureDifference DeltaT_sec_des(min=1) =   20
            "desired temperature difference secondary side"
            annotation(Dialog(group="Temperature objectives"));
        parameter Real V_dot_sec_max(unit="l/min", displayUnit="l/min") = 8.5
          "maximum secondary side volume flow in [l/min]"
          annotation(Dialog(group="General PID settings"));
        parameter Real k_prim_prod = 1.5
          "Proportional gain for controller in [-]"
          annotation(Dialog(group="PID primary side - producer mode - tuning"));
        parameter Real Ti_prim_prod = 8
          "Integral time constant for controller in [s]"
          annotation(Dialog(group="PID primary side - producer mode - tuning"));
        parameter Real Td_prim_prod = 0
          "Derivative time constant for controller in [s]"
          annotation(Dialog(group="PID primary side - producer mode - tuning"));
        parameter Real alpha_prim_prod(min=0, max=1) = 0.666
          "weight for the relevance of the error of the transferred heat in comparison to the error of temperature objectives (sum is one)"
          annotation(Dialog(group="PID primary side - producer mode - tuning"));
        parameter Real k_sec_prod = 1.5
          "Proportional gain for controller in [-]"
          annotation(Dialog(group="PID secondary side - producer mode - tuning"));
        parameter Real Ti_sec_prod = 8
          "Integral time constant for controller in [s]"
          annotation(Dialog(group="PID secondary side - producer mode - tuning"));
        parameter Real Td_sec_prod = 0
          "Derivative time constant for controller in [s]"
          annotation(Dialog(group="PID secondary side - producer mode - tuning"));
        parameter Real alpha_sec_prod(min=0, max=1) = 0.333
          "weight for the relevance of the error of the transferred heat in comparison to the error of temperature objectives (sum is one)"
          annotation(Dialog(group="PID secondary side - producer mode - tuning"));
        parameter Real k_prim_cons = 1
          "Proportional gain for controller in [-]"
          annotation(Dialog(group="PID primary side - consumer mode - tuning"));
        parameter Real Ti_prim_cons = 35
          "Integral time constant for controller in [s]"
          annotation(Dialog(group="PID primary side - consumer mode - tuning"));
        parameter Real Td_prim_cons = 0
          "Derivative time constant for controller in [s]"
          annotation(Dialog(group="PID primary side - consumer mode - tuning"));
        parameter Real alpha_prim_cons(min=0, max=1) = 0.333
          "weight for the relevance of the error of the transferred heat in comparison to the error of temperature objectives (sum is one)"
          annotation(Dialog(group="PID primary side - consumer mode - tuning"));
        parameter Real k_sec_cons = 1.5
          "Proportional gain for controller in [-]"
          annotation(Dialog(group="PID secondary side - consumer mode - tuning"));
        parameter Real Ti_sec_cons = 8
          "Integral time constant for controller in [s]"
          annotation(Dialog(group="PID secondary side - consumer mode - tuning"));
        parameter Real Td_sec_cons = 0
          "Derivative time constant for controller in [s]"
          annotation(Dialog(group="PID secondary side - consumer mode - tuning"));
        parameter Real alpha_sec_cons(min=0, max=1) = 0.666
          "weight for the relevance of the error of the transferred heat in comparison to the error of temperature objectives (sum is one)"
          annotation(Dialog(group="PID secondary side - consumer mode - tuning"));
        parameter .Modelica.Blocks.Types.SimpleController controllerType=
               Modelica.Blocks.Types.SimpleController.PID "Type of controller"
               annotation(Dialog(tab="Advanced", group="PIDs general"));
        parameter Init initType = Modelica.Blocks.Types.Init.NoInit
          "Type of initialization (1: no init, 2: steady state, 3: initial state, 4: initial output)"
          annotation(Dialog(tab="Advanced", group="PIDs general"));
        parameter Real tol = 0.1
          "tolerance [kW] for idle mode concerning heat transfer setpoint dotQ"
          annotation(Dialog(tab="Advanced", group="Miscellaneous"));

        // !!!!! variables !!!!!
        Real beta_prim_prod(min=0, max=1)
          "weight for the relevance of the error of the temperature(difference)";
        Real beta_sec_prod(min=0, max=1)
          "weight for the relevance of the error of the temperature(difference)";
        Real beta_prim_cons(min=0, max=1)
          "weight for the relevance of the error of the temperature(difference)";
        Real beta_sec_cons(min=0, max=1)
          "weight for the relevance of the error of the temperature(difference)";
        Integer  prosumer_mode
          "prosumer mode {-1;0;1}";
        Real T_prim_relev_des
            "desired value of relevant temperature (difference)
      for control of primary side";
        Real T_prim_relev_is
            "current value of relevant temperature (difference)
      for control of primary side";
        Real T_sec_relev_des
            "desired value of relevant temperature (difference)
      for control of secondary side";
        Real T_sec_relev_is
            "current value of relevant temperature (difference)
      for control of secondary side";

        Real PIDin_prim_cons_is_weighted
            "weighted input of is-values for PID_prim_cons";
        Real PIDin_prim_cons_des_weighted
            "weighted input of desired values for PID_prim_cons";
        Real PIDin_prim_prod_is_weighted
            "weighted input of is-values for PID_prim_prod";
        Real PIDin_prim_prod_des_weighted
            "weighted input of desired values for PID_prim_prod";
        Real PIDin_sec_cons_is_weighted
            "weighted input of is-values for PID_sec_cons";
        Real PIDin_sec_cons_des_weighted
            "weighted input of desired values for PID_sec_cons";
        Real PIDin_sec_prod_is_weighted
            "weighted input of is-values for PID_sec_prod";
        Real PIDin_sec_prod_des_weighted
            "weighted input of desired values for PID_sec_prod";

        Real error_prim_weighted
            "weighted overall error of primary side controller";

        Real error_sec_weighted
            "weighted overall error of primary side controller";

        Real error_T_prim_abs
            "temperature error of primary side controller";

        Real error_T_sec_abs
            "temperature error of primary side controller";

        Real error_Q_abs
            "temperature error of primary side controller";

        Real error_T_high_prio_abs
            "error of higher prioritized temperature objective";

        Real error_T_low_prio_abs
            "error of lower prioritized temperature objective";

        Real Delta_T_prim
            "weighted overall error of primary side controller";

        Real Delta_T_sec
            "weighted overall error of primary side controller";

        Real Q_dot_is_use;
        Real Q_dot_set_use;

         Real Delta_p_prim(unit="Pa", displayUnit="bar") annotation (
            Placement(transformation(
              extent={{-20,-20},{20,20}},
              rotation=180,
              origin={-80,-140})));

        // !!!!! ports !!!!!

        Modelica.Blocks.Interfaces.RealVectorInput states[8]
          annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));

        Modelica.Blocks.Interfaces.RealVectorOutput contr_vars_real[6]
          annotation (Placement(transformation(extent={{100,-20},{140,20}})));

        Real u_set
          "Normalized velocity of feed-in pump"
            annotation (Placement(transformation(
              extent={{-20,-20},{20,20}},
              rotation=0,
              origin={80,-60}),  iconTransformation(
              extent={{-20,-20},{20,20}},
              rotation=0,
              origin={100,-60})));
        Real kappa_set
          "Normalized flow coefficient for control valve"
           annotation (Placement(transformation(
              extent={{-20,-20},{20,20}},
              rotation=0,
              origin={80,-100}),iconTransformation(
              extent={{-20,-20},{20,20}},
              rotation=0,
              origin={100,-100})));
        Real pi_set
          "Participation" annotation (Placement(transformation(
              extent={{-20,-20},{20,20}},
              rotation=0,
              origin={80,20}),    iconTransformation(
              extent={{-20,-20},{20,20}},
              rotation=0,
              origin={100,20})));
        Real mu_set
          "Operating mode" annotation (Placement(transformation(
              extent={{-20,-20},{20,20}},
              rotation=0,
              origin={80,-20}),  iconTransformation(
              extent={{-20,-20},{20,20}},
              rotation=0,
              origin={100,-20})));
        Real T_sec_hot(
          unit="K",
          displayUnit="degC",
          min=277) "current temperature hot level secondary side"      annotation (Placement(
              transformation(
              extent={{-20,-20},{20,20}},
              rotation=0,
              origin={-80,60})));
        Real T_sec_cold(
          unit="K",
          displayUnit="degC",
          min=277)
                 "current temperature cold  level secondary side"    annotation (Placement(
              transformation(
              extent={{-20,-20},{20,20}},
              rotation=0,
              origin={-80,20})));
        Real T_prim_hot(
          unit="K",
          displayUnit="degC",
          min=277)
                 "current temperature hot level primary side"        annotation (Placement(
              transformation(
              extent={{-20,-20},{20,20}},
              rotation=0,
              origin={-80,140})));
        Real T_prim_cold(
          unit="K",
          displayUnit="degC",
          min=277)
                  "current temperature cold level primary side"       annotation (Placement(
              transformation(
              extent={{-20,-20},{20,20}},
              rotation=0,
              origin={-80,100})));
        Modelica.Blocks.Interfaces.RealInput Q_dot_set(unit="kW", displayUnit="kW")
          "setpoint heat transfer (positive production, negative consumption)"
          annotation (Placement(transformation(extent={{-20,-20},{20,20}},
              rotation=-90,
              origin={-60,188})));
        Real V_dot_prim(unit="l/min", displayUnit="l/min")
          annotation (Placement(transformation(extent={{-100,-40},{-60,0}})));
        Real V_dot_sec(unit="l/min", displayUnit="l/min")
          annotation (Placement(transformation(extent={{-100,-80},{-60,-40}})));

        Real Q_dot_is(unit="kW", displayUnit="kW")
          "currently transferred heat (positive production, negative consumption)"
          annotation (Placement(transformation(extent={{-100,-120},{-60,-80}})));
        Modelica.Blocks.Continuous.LimPID PID_prim_cons(
          controllerType=controllerType,
          k=k_prim_cons,
          Ti=Ti_prim_cons,
          Td=Td_prim_cons,
          yMax=1,
          yMin=0.25,
          initType=initType,
          y_start=PID_prim_cons.yMax)
          annotation (Placement(transformation(extent={{-30,-32},{-10,-12}})));
        Modelica.Blocks.Continuous.LimPID PID_sec_cons(
          controllerType=controllerType,
          k=k_sec_cons,
          Ti=Ti_sec_cons,
          Td=Td_sec_cons,
          yMax=V_dot_sec_max,
          yMin=2,
          initType=initType,
          y_start=PID_sec_cons.yMax)
          annotation (Placement(transformation(extent={{-34,28},{-14,48}})));
        Modelica.Blocks.Continuous.LimPID PID_prim_prod(
          controllerType=controllerType,
          k=k_prim_prod,
          Ti=Ti_prim_prod,
          Td=Td_prim_prod,
          yMax=1,
          yMin=0.25,
          initType=initType,
          y_start=PID_prim_prod.yMax) annotation (Placement(transformation(extent={{12,-30},
                  {32,-10}})));
        Modelica.Blocks.Continuous.LimPID PID_sec_prod(
          controllerType=controllerType,
          k=k_sec_prod,
          Ti=Ti_sec_prod,
          Td=Td_sec_prod,
          yMax=V_dot_sec_max,
          yMin=2,
          initType=initType,
          y_start=PID_sec_prod.yMax) annotation (Placement(transformation(extent={{10,28},
                  {30,48}})));
        Real T_sec_set(unit="K", displayUnit="degC")
           "Temperature on the secondary side" annotation (Placement(transformation(
              extent={{-20,-20},{20,20}},
              rotation=0,
              origin={80,100})));
        Real V_dot_sec_set(unit="l/min", displayUnit=
             "l/min") "volume flow rate setpoint on the secondary side" annotation (Placement(transformation(
              extent={{-20,-20},{20,20}},
              rotation=0,
              origin={80,60})));
        Modelica.Blocks.Interfaces.RealInput T_sec_in_is(unit="K", displayUnit="degC")
          "Temperature on the secondary side" annotation (Placement(transformation(
              extent={{-20,-20},{20,20}},
              rotation=270,
              origin={60,190})));

      equation

        // assign inputs
        T_prim_hot   = states[1];
        T_prim_cold  = states[2];
        T_sec_hot    = states[3];
        T_sec_cold   = states[4];
        V_dot_prim   = states[5];
        V_dot_sec    = states[6];
        Q_dot_is     = states[7];
        Delta_p_prim = states[8];

        Delta_T_prim      = T_prim_hot -T_prim_cold;
        Delta_T_sec       = T_sec_hot  -T_sec_cold;

        beta_prim_prod = 1 - alpha_prim_prod;
        beta_sec_prod  = 1 - alpha_sec_prod;
        beta_prim_cons = 1 - alpha_prim_cons;
        beta_sec_cons  = 1 - alpha_sec_cons;

        // determine easy static values that just depend on prosumer mode
        // determine inputs for the four PIDs
        // four PIDs in order to be able to have different gains for each situation

        if  Q_dot_set <= 0-tol then // consumption mode
          prosumer_mode = -1;
        elseif Q_dot_set >= 0+tol then // production mode
          prosumer_mode = +1;
        else // idle mode
          prosumer_mode = 0;
        end if;

        Q_dot_is_use = abs(Q_dot_is);
        Q_dot_set_use = abs(Q_dot_set);

        if prosumer_mode == -1 then // consumption mode
          pi_set = 1;
          mu_set = -1;
          T_prim_relev_des = T_sec_hot_des;
          T_prim_relev_is = T_sec_hot;
          T_sec_relev_des = DeltaT_prim_des;
          T_sec_relev_is = T_prim_hot-T_prim_cold;

          PIDin_prim_cons_is_weighted    = alpha_prim_cons*Q_dot_is_use/Delta_Qdot_norm + beta_prim_cons*T_prim_relev_is/Delta_T_norm;
          PIDin_prim_cons_des_weighted   = alpha_prim_cons*Q_dot_set_use/Delta_Qdot_norm + beta_prim_cons*T_prim_relev_des/Delta_T_norm;
          PIDin_prim_prod_is_weighted    = 0;
          PIDin_prim_prod_des_weighted   = 0;
          PIDin_sec_cons_is_weighted     = alpha_sec_cons*Q_dot_is_use/Delta_Qdot_norm + beta_sec_cons*T_sec_relev_is/Delta_T_norm;
          PIDin_sec_cons_des_weighted    = alpha_sec_cons*Q_dot_set_use/Delta_Qdot_norm +
            beta_sec_cons*T_sec_relev_des/Delta_T_norm;
          PIDin_sec_prod_is_weighted     = 0;
          PIDin_sec_prod_des_weighted    = 0;

          error_prim_weighted            = PIDin_prim_cons_des_weighted - PIDin_prim_cons_is_weighted;
          error_sec_weighted             = PIDin_sec_cons_des_weighted - PIDin_sec_cons_is_weighted;

          error_T_prim_abs               = DeltaT_prim_des - Delta_T_prim;
          error_T_sec_abs                = T_sec_hot_des - T_sec_hot;

          error_T_high_prio_abs          = T_sec_hot_des - T_sec_hot; // T_sec_hot
          error_T_low_prio_abs           = DeltaT_prim_des - Delta_T_prim; // Delta_T_prim

        elseif prosumer_mode == 1 then // production mode
          pi_set = 1;
          mu_set = 1;
          T_prim_relev_des = DeltaT_sec_des;
          T_prim_relev_is = T_sec_hot-T_sec_cold;
          T_sec_relev_des = T_prim_hot_des;
          T_sec_relev_is = T_prim_hot;

          PIDin_prim_cons_is_weighted    = 0;
          PIDin_prim_cons_des_weighted   = 0;
          PIDin_prim_prod_is_weighted    = alpha_prim_prod*Q_dot_is_use/Delta_Qdot_norm + beta_prim_prod*T_prim_relev_is/Delta_T_norm;
          PIDin_prim_prod_des_weighted   = alpha_prim_prod*Q_dot_set_use/Delta_Qdot_norm + beta_prim_prod*T_prim_relev_des/Delta_T_norm;
          PIDin_sec_cons_is_weighted     = 0;
          PIDin_sec_cons_des_weighted    = 0;
          PIDin_sec_prod_is_weighted     = alpha_sec_prod*Q_dot_is_use/Delta_Qdot_norm + beta_sec_prod*T_sec_relev_is/Delta_T_norm;
          PIDin_sec_prod_des_weighted    = alpha_sec_prod*Q_dot_set_use/Delta_Qdot_norm + beta_sec_prod*T_sec_relev_des/Delta_T_norm;

          error_prim_weighted            = PIDin_prim_prod_des_weighted - PIDin_prim_prod_is_weighted;
          error_sec_weighted             = PIDin_sec_prod_des_weighted - PIDin_sec_prod_is_weighted;

          error_T_prim_abs               = T_prim_hot_des - T_prim_hot;
          error_T_sec_abs                = DeltaT_sec_des - Delta_T_sec;

          error_T_high_prio_abs          = T_prim_hot_des - T_prim_hot; // T_pim_hot
          error_T_low_prio_abs           = DeltaT_sec_des - Delta_T_sec; // Deltat_T_sec

        else // idle mode
          pi_set = 0;
          mu_set = -1;
          T_prim_relev_des = 0;
          T_prim_relev_is = 0;
          T_sec_relev_des = 0;
          T_sec_relev_is = 0;

          PIDin_prim_cons_is_weighted    = 0;
          PIDin_prim_cons_des_weighted   = 0;
          PIDin_prim_prod_is_weighted    = 0;
          PIDin_prim_prod_des_weighted   = 0;
          PIDin_sec_cons_is_weighted     = 0;
          PIDin_sec_cons_des_weighted    = 0;
          PIDin_sec_prod_is_weighted     = 0;
          PIDin_sec_prod_des_weighted    = 0;

          error_prim_weighted            = 0;
          error_sec_weighted             = 0;

          error_T_prim_abs               = 0;
          error_T_sec_abs                = 0;

          error_T_high_prio_abs          = 0;
          error_T_low_prio_abs           = 0;

        end if;

        // assign PID controller inputs
        PID_prim_cons.u_s    = PIDin_prim_cons_des_weighted;
        PID_prim_cons.u_m    = PIDin_prim_cons_is_weighted;
        PID_prim_prod.u_s    = PIDin_prim_prod_des_weighted;
        PID_prim_prod.u_m    = PIDin_prim_prod_is_weighted;
        PID_sec_cons.u_s     = PIDin_sec_cons_des_weighted;
        PID_sec_cons.u_m     = PIDin_sec_cons_is_weighted;
        PID_sec_prod.u_s     = PIDin_sec_prod_des_weighted;
        PID_sec_prod.u_m     = PIDin_sec_prod_is_weighted;

        // connect secondary side temperature setpoint
        T_sec_set    =  T_sec_in_is;

        // assign PID outputs to controller outputs
        if prosumer_mode == -1 then // consumption mode
          u_set = 0;
          kappa_set =PID_prim_cons.y;
          V_dot_sec_set = PID_sec_cons.y;
        elseif prosumer_mode == 1 then // production mode
          u_set =PID_prim_prod.y;
          kappa_set = 0;
          V_dot_sec_set = PID_sec_prod.y;
        else // idle mode
          V_dot_sec_set = 0;
          u_set = 0;
          kappa_set = 0;
        end if;

        error_Q_abs = Q_dot_set_use - Q_dot_is_use;

        // assign control variables vector
        contr_vars_real[1]   =  T_sec_set;
        contr_vars_real[2]   =  V_dot_sec_set;
        contr_vars_real[3]   =  pi_set;
        contr_vars_real[4]   =  mu_set;
        contr_vars_real[5]   =  u_set;
        contr_vars_real[6]   =  kappa_set;

          annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-120,-160},{
                  120,180}}),                                           graphics={
                Text(
                extent={{-70,56},{64,-56}},
                textColor={28,108,200},
                textString="weighted
PID"),       Rectangle(extent={{-120,180},{120,-160}}, lineColor={0,0,0})}),
                                                                         Diagram(
              coordinateSystem(preserveAspectRatio=false, extent={{-120,-160},{120,180}})));
      end PID_Q_T_weighted_crossover;

      model PID_Q_T_weighted_mix1

        import Modelica.Units.SI;
        import T_AbsZeroDegC = Modelica.Constants.T_zero;
        import Modelica.Blocks.Types.Init;
        import Modelica.Blocks.Types.SimpleController;

          // !!!!! parameters !!!!!
        parameter Real Delta_Qdot_norm = 1
            "Heat power value for normalizing the error (deviation) of the transferred heat.
      For alpha=0.5 a deviation of Delta_Qdot_norm in heat transfer is weigthed equal to a deviation of Delta_T_norm in temperature."
            annotation(Dialog(group="Normalizing values"));
        parameter SI.TemperatureDifference Delta_T_norm(min=0) = 3
            "Temperature difference for normalizing the error (deviation) of the temperature.
      For alpha=0.5 a deviation of Delta_T_norm in temperature is weighted equal to a deviation of Delta_Qdot_norm in heat transfer."
            annotation(Dialog(group="Normalizing values"));

        parameter SI.Temperature T_prim_hot_des(min=277)= - T_AbsZeroDegC + 53.5
            "desired temperature supply primary side"
            annotation(Dialog(group="Temperature objectives"));
        parameter SI.Temperature T_sec_hot_des(min=277)= - T_AbsZeroDegC + 50
            "desired temperature supply secondary side"
            annotation(Dialog(group="Temperature objectives"));
        parameter SI.TemperatureDifference DeltaT_prim_des(min=1) =   20
            "desired temperature difference primary side"
            annotation(Dialog(group="Temperature objectives"));
        parameter SI.TemperatureDifference DeltaT_sec_des(min=1) =   20
            "desired temperature difference secondary side"
            annotation(Dialog(group="Temperature objectives"));
        parameter Real V_dot_sec_max(unit="l/min", displayUnit="l/min") = 8.5
          "maximum secondary side volume flow in [l/min]"
          annotation(Dialog(group="General PID settings"));
        parameter Real k_prim_prod = 1.5
          "Proportional gain for controller in [-]"
          annotation(Dialog(group="PID primary side - producer mode - tuning"));
        parameter Real Ti_prim_prod = 8
          "Integral time constant for controller in [s]"
          annotation(Dialog(group="PID primary side - producer mode - tuning"));
        parameter Real Td_prim_prod = 0
          "Derivative time constant for controller in [s]"
          annotation(Dialog(group="PID primary side - producer mode - tuning"));
        parameter Real alpha_prim_prod(min=0, max=1) = 0.333
          "weight for the relevance of the error of the transferred heat in comparison to the error of temperature objectives (sum is one)"
          annotation(Dialog(group="PID primary side - producer mode - tuning"));
        parameter Real k_sec_prod = 1.5
          "Proportional gain for controller in [-]"
          annotation(Dialog(group="PID secondary side - producer mode - tuning"));
        parameter Real Ti_sec_prod = 8
          "Integral time constant for controller in [s]"
          annotation(Dialog(group="PID secondary side - producer mode - tuning"));
        parameter Real Td_sec_prod = 0
          "Derivative time constant for controller in [s]"
          annotation(Dialog(group="PID secondary side - producer mode - tuning"));
        parameter Real alpha_sec_prod(min=0, max=1) = 0.666
          "weight for the relevance of the error of the transferred heat in comparison to the error of temperature objectives (sum is one)"
          annotation(Dialog(group="PID secondary side - producer mode - tuning"));
        parameter Real k_prim_cons = 1.0
          "Proportional gain for controller in [-]"
          annotation(Dialog(group="PID primary side - consumer mode - tuning"));
        parameter Real Ti_prim_cons = 35
          "Integral time constant for controller in [s]"
          annotation(Dialog(group="PID primary side - consumer mode - tuning"));
        parameter Real Td_prim_cons = 0
          "Derivative time constant for controller in [s]"
          annotation(Dialog(group="PID primary side - consumer mode - tuning"));
        parameter Real alpha_prim_cons(min=0, max=1) = 0.333
          "weight for the relevance of the error of the transferred heat in comparison to the error of temperature objectives (sum is one)"
          annotation(Dialog(group="PID primary side - consumer mode - tuning"));
        parameter Real k_sec_cons = 1.5
          "Proportional gain for controller in [-]"
          annotation(Dialog(group="PID secondary side - consumer mode - tuning"));
        parameter Real Ti_sec_cons = 8
          "Integral time constant for controller in [s]"
          annotation(Dialog(group="PID secondary side - consumer mode - tuning"));
        parameter Real Td_sec_cons = 0
          "Derivative time constant for controller in [s]"
          annotation(Dialog(group="PID secondary side - consumer mode - tuning"));
        parameter Real alpha_sec_cons(min=0, max=1) = 0.666
          "weight for the relevance of the error of the transferred heat in comparison to the error of temperature objectives (sum is one)"
          annotation(Dialog(group="PID secondary side - consumer mode - tuning"));
        parameter .Modelica.Blocks.Types.SimpleController controllerType=
               Modelica.Blocks.Types.SimpleController.PID "Type of controller"
               annotation(Dialog(tab="Advanced", group="PIDs general"));
        parameter Init initType = Modelica.Blocks.Types.Init.NoInit
          "Type of initialization (1: no init, 2: steady state, 3: initial state, 4: initial output)"
          annotation(Dialog(tab="Advanced", group="PIDs general"));
        parameter Real tol = 0.1
          "tolerance [kW] for idle mode concerning heat transfer setpoint dotQ"
          annotation(Dialog(tab="Advanced", group="Miscellaneous"));

        // !!!!! variables !!!!!
        Real beta_prim_prod(min=0, max=1)
          "weight for the relevance of the error of the temperature(difference)";
        Real beta_sec_prod(min=0, max=1)
          "weight for the relevance of the error of the temperature(difference)";
        Real beta_prim_cons(min=0, max=1)
          "weight for the relevance of the error of the temperature(difference)";
        Real beta_sec_cons(min=0, max=1)
          "weight for the relevance of the error of the temperature(difference)";
        Integer  prosumer_mode
          "prosumer mode {-1;0;1}";
        Real T_prim_relev_des
            "desired value of relevant temperature (difference)
      for control of primary side";
        Real T_prim_relev_is
            "current value of relevant temperature (difference)
      for control of primary side";
        Real T_sec_relev_des
            "desired value of relevant temperature (difference)
      for control of secondary side";
        Real T_sec_relev_is
            "current value of relevant temperature (difference)
      for control of secondary side";

        Real PIDin_prim_cons_is_weighted
            "weighted input of is-values for PID_prim_cons";
        Real PIDin_prim_cons_des_weighted
            "weighted input of desired values for PID_prim_cons";
        Real PIDin_prim_prod_is_weighted
            "weighted input of is-values for PID_prim_prod";
        Real PIDin_prim_prod_des_weighted
            "weighted input of desired values for PID_prim_prod";
        Real PIDin_sec_cons_is_weighted
            "weighted input of is-values for PID_sec_cons";
        Real PIDin_sec_cons_des_weighted
            "weighted input of desired values for PID_sec_cons";
        Real PIDin_sec_prod_is_weighted
            "weighted input of is-values for PID_sec_prod";
        Real PIDin_sec_prod_des_weighted
            "weighted input of desired values for PID_sec_prod";

        Real error_prim_weighted
            "weighted overall error of primary side controller";

        Real error_sec_weighted
            "weighted overall error of primary side controller";

        Real error_T_prim_abs
            "temperature error of primary side controller";

        Real error_T_sec_abs
            "temperature error of primary side controller";

        Real error_Q_abs
            "temperature error of primary side controller";

        Real error_T_high_prio_abs
            "error of higher prioritized temperature objective";

        Real error_T_low_prio_abs
            "error of lower prioritized temperature objective";

        Real Delta_T_prim
            "weighted overall error of primary side controller";

        Real Delta_T_sec
            "weighted overall error of primary side controller";

        Real Q_dot_is_use;
        Real Q_dot_set_use;

         Real Delta_p_prim(unit="Pa", displayUnit="bar") annotation (
            Placement(transformation(
              extent={{-20,-20},{20,20}},
              rotation=180,
              origin={-80,-140})));

        // !!!!! ports !!!!!

        Modelica.Blocks.Interfaces.RealVectorInput states[8]
          annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));

        Modelica.Blocks.Interfaces.RealVectorOutput contr_vars_real[6]
          annotation (Placement(transformation(extent={{100,-20},{140,20}})));

        Real u_set
          "Normalized velocity of feed-in pump"
            annotation (Placement(transformation(
              extent={{-20,-20},{20,20}},
              rotation=0,
              origin={80,-60}),  iconTransformation(
              extent={{-20,-20},{20,20}},
              rotation=0,
              origin={100,-60})));
        Real kappa_set
          "Normalized flow coefficient for control valve"
           annotation (Placement(transformation(
              extent={{-20,-20},{20,20}},
              rotation=0,
              origin={80,-100}),iconTransformation(
              extent={{-20,-20},{20,20}},
              rotation=0,
              origin={100,-100})));
        Real pi_set
          "Participation" annotation (Placement(transformation(
              extent={{-20,-20},{20,20}},
              rotation=0,
              origin={80,20}),    iconTransformation(
              extent={{-20,-20},{20,20}},
              rotation=0,
              origin={100,20})));
        Real mu_set
          "Operating mode" annotation (Placement(transformation(
              extent={{-20,-20},{20,20}},
              rotation=0,
              origin={80,-20}),  iconTransformation(
              extent={{-20,-20},{20,20}},
              rotation=0,
              origin={100,-20})));
        Real T_sec_hot(
          unit="K",
          displayUnit="degC",
          min=277) "current temperature hot level secondary side"      annotation (Placement(
              transformation(
              extent={{-20,-20},{20,20}},
              rotation=0,
              origin={-80,60})));
        Real T_sec_cold(
          unit="K",
          displayUnit="degC",
          min=277)
                 "current temperature cold  level secondary side"    annotation (Placement(
              transformation(
              extent={{-20,-20},{20,20}},
              rotation=0,
              origin={-80,20})));
        Real T_prim_hot(
          unit="K",
          displayUnit="degC",
          min=277)
                 "current temperature hot level primary side"        annotation (Placement(
              transformation(
              extent={{-20,-20},{20,20}},
              rotation=0,
              origin={-80,140})));
        Real T_prim_cold(
          unit="K",
          displayUnit="degC",
          min=277)
                  "current temperature cold level primary side"       annotation (Placement(
              transformation(
              extent={{-20,-20},{20,20}},
              rotation=0,
              origin={-80,100})));
        Modelica.Blocks.Interfaces.RealInput Q_dot_set(unit="kW", displayUnit="kW")
          "setpoint heat transfer (positive production, negative consumption)"
          annotation (Placement(transformation(extent={{-20,-20},{20,20}},
              rotation=-90,
              origin={-60,188})));
        Real V_dot_prim(unit="l/min", displayUnit="l/min")
          annotation (Placement(transformation(extent={{-100,-40},{-60,0}})));
        Real V_dot_sec(unit="l/min", displayUnit="l/min")
          annotation (Placement(transformation(extent={{-100,-80},{-60,-40}})));

        Real Q_dot_is(unit="kW", displayUnit="kW")
          "currently transferred heat (positive production, negative consumption)"
          annotation (Placement(transformation(extent={{-100,-120},{-60,-80}})));
        Modelica.Blocks.Continuous.LimPID PID_prim_cons(
          controllerType=controllerType,
          k=k_prim_cons,
          Ti=Ti_prim_cons,
          Td=Td_prim_cons,
          yMax=1,
          yMin=0.25,
          initType=initType,
          y_start=PID_prim_cons.yMax)
          annotation (Placement(transformation(extent={{-30,-32},{-10,-12}})));
        Modelica.Blocks.Continuous.LimPID PID_sec_cons(
          controllerType=controllerType,
          k=k_sec_cons,
          Ti=Ti_sec_cons,
          Td=Td_sec_cons,
          yMax=V_dot_sec_max,
          yMin=2,
          initType=initType,
          y_start=PID_sec_cons.yMax)
          annotation (Placement(transformation(extent={{-34,28},{-14,48}})));
        Modelica.Blocks.Continuous.LimPID PID_prim_prod(
          controllerType=controllerType,
          k=k_prim_prod,
          Ti=Ti_prim_prod,
          Td=Td_prim_prod,
          yMax=1,
          yMin=0.25,
          initType=initType,
          y_start=PID_prim_prod.yMax) annotation (Placement(transformation(extent={{12,-30},
                  {32,-10}})));
        Modelica.Blocks.Continuous.LimPID PID_sec_prod(
          controllerType=controllerType,
          k=k_sec_prod,
          Ti=Ti_sec_prod,
          Td=Td_sec_prod,
          yMax=V_dot_sec_max,
          yMin=2,
          initType=initType,
          y_start=PID_sec_prod.yMax) annotation (Placement(transformation(extent={{10,28},
                  {30,48}})));
        Real T_sec_set(unit="K", displayUnit="degC")
           "Temperature on the secondary side" annotation (Placement(transformation(
              extent={{-20,-20},{20,20}},
              rotation=0,
              origin={80,100})));
        Real V_dot_sec_set(unit="l/min", displayUnit=
             "l/min") "volume flow rate setpoint on the secondary side" annotation (Placement(transformation(
              extent={{-20,-20},{20,20}},
              rotation=0,
              origin={80,60})));
        Modelica.Blocks.Interfaces.RealInput T_sec_in_is(unit="K", displayUnit="degC")
          "Temperature on the secondary side" annotation (Placement(transformation(
              extent={{-20,-20},{20,20}},
              rotation=270,
              origin={60,190})));

      equation

        // assign inputs
        T_prim_hot   = states[1];
        T_prim_cold  = states[2];
        T_sec_hot    = states[3];
        T_sec_cold   = states[4];
        V_dot_prim   = states[5];
        V_dot_sec    = states[6];
        Q_dot_is     = states[7];
        Delta_p_prim = states[8];

        Delta_T_prim      = T_prim_hot -T_prim_cold;
        Delta_T_sec       = T_sec_hot  -T_sec_cold;

        beta_prim_prod = 1 - alpha_prim_prod;
        beta_sec_prod  = 1 - alpha_sec_prod;
        beta_prim_cons = 1 - alpha_prim_cons;
        beta_sec_cons  = 1 - alpha_sec_cons;

        // determine easy static values that just depend on prosumer mode
        // determine inputs for the four PIDs
        // four PIDs in order to be able to have different gains for each situation

        if  Q_dot_set <= 0-tol then // consumption mode
          prosumer_mode = -1;
        elseif Q_dot_set >= 0+tol then // production mode
          prosumer_mode = +1;
        else // idle mode
          prosumer_mode = 0;
        end if;

        Q_dot_is_use = abs(Q_dot_is);
        Q_dot_set_use = abs(Q_dot_set);

        if prosumer_mode == -1 then // consumption mode
          pi_set = 1;
          mu_set = -1;
          T_prim_relev_des = T_sec_hot_des;
          T_prim_relev_is = T_sec_hot;
          T_sec_relev_des = DeltaT_prim_des;
          T_sec_relev_is = T_prim_hot-T_prim_cold;

          PIDin_prim_cons_is_weighted    = alpha_prim_cons*Q_dot_is_use/Delta_Qdot_norm + beta_prim_cons*T_prim_relev_is/Delta_T_norm;
          PIDin_prim_cons_des_weighted   = alpha_prim_cons*Q_dot_set_use/Delta_Qdot_norm + beta_prim_cons*T_prim_relev_des/Delta_T_norm;
          PIDin_prim_prod_is_weighted    = 0;
          PIDin_prim_prod_des_weighted   = 0;
          PIDin_sec_cons_is_weighted     = alpha_sec_cons*Q_dot_is_use/Delta_Qdot_norm + beta_sec_cons*T_sec_relev_is/Delta_T_norm;
          PIDin_sec_cons_des_weighted    = alpha_sec_cons*Q_dot_set_use/Delta_Qdot_norm + beta_sec_cons*T_sec_relev_des/Delta_T_norm;
          PIDin_sec_prod_is_weighted     = 0;
          PIDin_sec_prod_des_weighted    = 0;

          error_prim_weighted            = PIDin_prim_cons_des_weighted - PIDin_prim_cons_is_weighted;
          error_sec_weighted             = PIDin_sec_cons_des_weighted - PIDin_sec_cons_is_weighted;

          error_T_prim_abs               = DeltaT_prim_des - Delta_T_prim;
          error_T_sec_abs                = T_sec_hot_des - T_sec_hot;

          error_T_high_prio_abs          = T_sec_hot_des - T_sec_hot; // T_sec_hot
          error_T_low_prio_abs           = DeltaT_prim_des - Delta_T_prim; // Delta_T_prim

        elseif prosumer_mode == 1 then // production mode
          pi_set = 1;
          mu_set = 1;
          T_prim_relev_des = T_prim_hot_des;
          T_prim_relev_is = T_prim_hot;
          T_sec_relev_des = DeltaT_sec_des;
          T_sec_relev_is = T_sec_hot-T_sec_cold;

          PIDin_prim_cons_is_weighted    = 0;
          PIDin_prim_cons_des_weighted   = 0;
          PIDin_prim_prod_is_weighted    = alpha_prim_prod*Q_dot_is_use/Delta_Qdot_norm + beta_prim_prod*(-1)*T_prim_relev_is/Delta_T_norm;
          PIDin_prim_prod_des_weighted   = alpha_prim_prod*Q_dot_set_use/Delta_Qdot_norm + beta_prim_prod*(-1)*T_prim_relev_des/Delta_T_norm;
          PIDin_sec_cons_is_weighted     = 0;
          PIDin_sec_cons_des_weighted    = 0;
          PIDin_sec_prod_is_weighted     = alpha_sec_prod*Q_dot_is_use/Delta_Qdot_norm + beta_sec_prod*(-1)*T_sec_relev_is/Delta_T_norm;
          PIDin_sec_prod_des_weighted    = alpha_sec_prod*Q_dot_set_use/Delta_Qdot_norm + beta_sec_prod*(-1)*T_sec_relev_des/Delta_T_norm;

          error_prim_weighted            = PIDin_prim_prod_des_weighted - PIDin_prim_prod_is_weighted;
          error_sec_weighted             = PIDin_sec_prod_des_weighted - PIDin_sec_prod_is_weighted;

          error_T_prim_abs               = T_prim_hot_des - T_prim_hot;
          error_T_sec_abs                = DeltaT_sec_des - Delta_T_sec;

          error_T_high_prio_abs          = T_prim_hot_des - T_prim_hot;
          error_T_low_prio_abs           = DeltaT_sec_des - Delta_T_sec;

        else // idle mode
          pi_set = 0;
          mu_set = -1;
          T_prim_relev_des = 0;
          T_prim_relev_is = 0;
          T_sec_relev_des = 0;
          T_sec_relev_is = 0;

          PIDin_prim_cons_is_weighted    = 0;
          PIDin_prim_cons_des_weighted   = 0;
          PIDin_prim_prod_is_weighted    = 0;
          PIDin_prim_prod_des_weighted   = 0;
          PIDin_sec_cons_is_weighted     = 0;
          PIDin_sec_cons_des_weighted    = 0;
          PIDin_sec_prod_is_weighted     = 0;
          PIDin_sec_prod_des_weighted    = 0;

          error_prim_weighted            = 0;
          error_sec_weighted             = 0;

          error_T_prim_abs               = 0;
          error_T_sec_abs                = 0;

          error_T_high_prio_abs          = 0;
          error_T_low_prio_abs           = 0;

        end if;

        // assign PID controller inputs
        PID_prim_cons.u_s    = PIDin_prim_cons_des_weighted;
        PID_prim_cons.u_m    = PIDin_prim_cons_is_weighted;
        PID_prim_prod.u_s    = PIDin_prim_prod_des_weighted;
        PID_prim_prod.u_m    = PIDin_prim_prod_is_weighted;
        PID_sec_cons.u_s     = PIDin_sec_cons_des_weighted;
        PID_sec_cons.u_m     = PIDin_sec_cons_is_weighted;
        PID_sec_prod.u_s     = PIDin_sec_prod_des_weighted;
        PID_sec_prod.u_m     = PIDin_sec_prod_is_weighted;

        // connect secondary side temperature setpoint
        T_sec_set    =  T_sec_in_is;

        // assign PID outputs to controller outputs
        if prosumer_mode == -1 then // consumption mode
          u_set = 0;
          kappa_set =PID_prim_cons.y;
          V_dot_sec_set = PID_sec_cons.y;
        elseif prosumer_mode == 1 then // production mode
          u_set =PID_prim_prod.y;
          kappa_set = 0;
          V_dot_sec_set = PID_sec_prod.y;
        else // idle mode
          V_dot_sec_set = 0;
          u_set = 0;
          kappa_set = 0;
        end if;

        error_Q_abs = Q_dot_set_use - Q_dot_is_use;

        // assign control variables vector
        contr_vars_real[1]   =  T_sec_set;
        contr_vars_real[2]   =  V_dot_sec_set;
        contr_vars_real[3]   =  pi_set;
        contr_vars_real[4]   =  mu_set;
        contr_vars_real[5]   =  u_set;
        contr_vars_real[6]   =  kappa_set;

          annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-120,-160},{
                  120,180}}),                                           graphics={
                Text(
                extent={{-70,56},{64,-56}},
                textColor={28,108,200},
                textString="weighted
PID"),       Rectangle(extent={{-120,180},{120,-160}}, lineColor={0,0,0})}),
                                                                         Diagram(
              coordinateSystem(preserveAspectRatio=false, extent={{-120,-160},{120,180}})));
      end PID_Q_T_weighted_mix1;

      model PID_Q_T_weighted_mix2

        import Modelica.Units.SI;
        import T_AbsZeroDegC = Modelica.Constants.T_zero;
        import Modelica.Blocks.Types.Init;
        import Modelica.Blocks.Types.SimpleController;

          // !!!!! parameters !!!!!
        parameter Real Delta_Qdot_norm = 1
            "Heat power value for normalizing the error (deviation) of the transferred heat.
      For alpha=0.5 a deviation of Delta_Qdot_norm in heat transfer is weigthed equal to a deviation of Delta_T_norm in temperature."
            annotation(Dialog(group="Normalizing values"));
        parameter SI.TemperatureDifference Delta_T_norm(min=0) = 3
            "Temperature difference for normalizing the error (deviation) of the temperature.
      For alpha=0.5 a deviation of Delta_T_norm in temperature is weighted equal to a deviation of Delta_Qdot_norm in heat transfer."
            annotation(Dialog(group="Normalizing values"));

        parameter SI.Temperature T_prim_hot_des(min=277)= - T_AbsZeroDegC + 53.5
            "desired temperature supply primary side"
            annotation(Dialog(group="Temperature objectives"));
        parameter SI.Temperature T_sec_hot_des(min=277)= - T_AbsZeroDegC + 50
            "desired temperature supply secondary side"
            annotation(Dialog(group="Temperature objectives"));
        parameter SI.TemperatureDifference DeltaT_prim_des(min=1) =   20
            "desired temperature difference primary side"
            annotation(Dialog(group="Temperature objectives"));
        parameter SI.TemperatureDifference DeltaT_sec_des(min=1) =   20
            "desired temperature difference secondary side"
            annotation(Dialog(group="Temperature objectives"));
        parameter Real V_dot_sec_max(unit="l/min", displayUnit="l/min") = 8.5
          "maximum secondary side volume flow in [l/min]"
          annotation(Dialog(group="General PID settings"));
        parameter Real k_prim_prod = 1.5
          "Proportional gain for controller in [-]"
          annotation(Dialog(group="PID primary side - producer mode - tuning"));
        parameter Real Ti_prim_prod = 8
          "Integral time constant for controller in [s]"
          annotation(Dialog(group="PID primary side - producer mode - tuning"));
        parameter Real Td_prim_prod = 0
          "Derivative time constant for controller in [s]"
          annotation(Dialog(group="PID primary side - producer mode - tuning"));
        parameter Real alpha_prim_prod(min=0, max=1) = 0.666
          "weight for the relevance of the error of the transferred heat in comparison to the error of temperature objectives (sum is one)"
          annotation(Dialog(group="PID primary side - producer mode - tuning"));
        parameter Real k_sec_prod = 1.5
          "Proportional gain for controller in [-]"
          annotation(Dialog(group="PID secondary side - producer mode - tuning"));
        parameter Real Ti_sec_prod = 8
          "Integral time constant for controller in [s]"
          annotation(Dialog(group="PID secondary side - producer mode - tuning"));
        parameter Real Td_sec_prod = 0
          "Derivative time constant for controller in [s]"
          annotation(Dialog(group="PID secondary side - producer mode - tuning"));
        parameter Real alpha_sec_prod(min=0, max=1) = 0.333
          "weight for the relevance of the error of the transferred heat in comparison to the error of temperature objectives (sum is one)"
          annotation(Dialog(group="PID secondary side - producer mode - tuning"));
        parameter Real k_prim_cons = 1.0
          "Proportional gain for controller in [-]"
          annotation(Dialog(group="PID primary side - consumer mode - tuning"));
        parameter Real Ti_prim_cons = 35
          "Integral time constant for controller in [s]"
          annotation(Dialog(group="PID primary side - consumer mode - tuning"));
        parameter Real Td_prim_cons = 0
          "Derivative time constant for controller in [s]"
          annotation(Dialog(group="PID primary side - consumer mode - tuning"));
        parameter Real alpha_prim_cons(min=0, max=1) = 0.666
          "weight for the relevance of the error of the transferred heat in comparison to the error of temperature objectives (sum is one)"
          annotation(Dialog(group="PID primary side - consumer mode - tuning"));
        parameter Real k_sec_cons = 1.5
          "Proportional gain for controller in [-]"
          annotation(Dialog(group="PID secondary side - consumer mode - tuning"));
        parameter Real Ti_sec_cons = 8
          "Integral time constant for controller in [s]"
          annotation(Dialog(group="PID secondary side - consumer mode - tuning"));
        parameter Real Td_sec_cons = 0
          "Derivative time constant for controller in [s]"
          annotation(Dialog(group="PID secondary side - consumer mode - tuning"));
        parameter Real alpha_sec_cons(min=0, max=1) = 0.333
          "weight for the relevance of the error of the transferred heat in comparison to the error of temperature objectives (sum is one)"
          annotation(Dialog(group="PID secondary side - consumer mode - tuning"));
        parameter .Modelica.Blocks.Types.SimpleController controllerType=
               Modelica.Blocks.Types.SimpleController.PID "Type of controller"
               annotation(Dialog(tab="Advanced", group="PIDs general"));
        parameter Init initType = Modelica.Blocks.Types.Init.NoInit
          "Type of initialization (1: no init, 2: steady state, 3: initial state, 4: initial output)"
          annotation(Dialog(tab="Advanced", group="PIDs general"));
        parameter Real tol = 0.1
          "tolerance [kW] for idle mode concerning heat transfer setpoint dotQ"
          annotation(Dialog(tab="Advanced", group="Miscellaneous"));

        // !!!!! variables !!!!!
        Real beta_prim_prod(min=0, max=1)
          "weight for the relevance of the error of the temperature(difference)";
        Real beta_sec_prod(min=0, max=1)
          "weight for the relevance of the error of the temperature(difference)";
        Real beta_prim_cons(min=0, max=1)
          "weight for the relevance of the error of the temperature(difference)";
        Real beta_sec_cons(min=0, max=1)
          "weight for the relevance of the error of the temperature(difference)";
        Integer  prosumer_mode
          "prosumer mode {-1;0;1}";
        Real T_prim_relev_des
            "desired value of relevant temperature (difference)
      for control of primary side";
        Real T_prim_relev_is
            "current value of relevant temperature (difference)
      for control of primary side";
        Real T_sec_relev_des
            "desired value of relevant temperature (difference)
      for control of secondary side";
        Real T_sec_relev_is
            "current value of relevant temperature (difference)
      for control of secondary side";

        Real PIDin_prim_cons_is_weighted
            "weighted input of is-values for PID_prim_cons";
        Real PIDin_prim_cons_des_weighted
            "weighted input of desired values for PID_prim_cons";
        Real PIDin_prim_prod_is_weighted
            "weighted input of is-values for PID_prim_prod";
        Real PIDin_prim_prod_des_weighted
            "weighted input of desired values for PID_prim_prod";
        Real PIDin_sec_cons_is_weighted
            "weighted input of is-values for PID_sec_cons";
        Real PIDin_sec_cons_des_weighted
            "weighted input of desired values for PID_sec_cons";
        Real PIDin_sec_prod_is_weighted
            "weighted input of is-values for PID_sec_prod";
        Real PIDin_sec_prod_des_weighted
            "weighted input of desired values for PID_sec_prod";

        Real error_prim_weighted
            "weighted overall error of primary side controller";

        Real error_sec_weighted
            "weighted overall error of primary side controller";

        Real error_T_prim_abs
            "temperature error of primary side controller";

        Real error_T_sec_abs
            "temperature error of primary side controller";

        Real error_Q_abs
            "temperature error of primary side controller";

        Real error_T_high_prio_abs
            "error of higher prioritized temperature objective";

        Real error_T_low_prio_abs
            "error of lower prioritized temperature objective";

        Real Delta_T_prim
            "weighted overall error of primary side controller";

        Real Delta_T_sec
            "weighted overall error of primary side controller";

        Real Q_dot_is_use;
        Real Q_dot_set_use;

         Real Delta_p_prim(unit="Pa", displayUnit="bar") annotation (
            Placement(transformation(
              extent={{-20,-20},{20,20}},
              rotation=180,
              origin={-80,-140})));

        // !!!!! ports !!!!!

        Modelica.Blocks.Interfaces.RealVectorInput states[8]
          annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));

        Modelica.Blocks.Interfaces.RealVectorOutput contr_vars_real[6]
          annotation (Placement(transformation(extent={{100,-20},{140,20}})));

        Real u_set
          "Normalized velocity of feed-in pump"
            annotation (Placement(transformation(
              extent={{-20,-20},{20,20}},
              rotation=0,
              origin={80,-60}),  iconTransformation(
              extent={{-20,-20},{20,20}},
              rotation=0,
              origin={100,-60})));
        Real kappa_set
          "Normalized flow coefficient for control valve"
           annotation (Placement(transformation(
              extent={{-20,-20},{20,20}},
              rotation=0,
              origin={80,-100}),iconTransformation(
              extent={{-20,-20},{20,20}},
              rotation=0,
              origin={100,-100})));
        Real pi_set
          "Participation" annotation (Placement(transformation(
              extent={{-20,-20},{20,20}},
              rotation=0,
              origin={80,20}),    iconTransformation(
              extent={{-20,-20},{20,20}},
              rotation=0,
              origin={100,20})));
        Real mu_set
          "Operating mode" annotation (Placement(transformation(
              extent={{-20,-20},{20,20}},
              rotation=0,
              origin={80,-20}),  iconTransformation(
              extent={{-20,-20},{20,20}},
              rotation=0,
              origin={100,-20})));
        Real T_sec_hot(
          unit="K",
          displayUnit="degC",
          min=277) "current temperature hot level secondary side"      annotation (Placement(
              transformation(
              extent={{-20,-20},{20,20}},
              rotation=0,
              origin={-80,60})));
        Real T_sec_cold(
          unit="K",
          displayUnit="degC",
          min=277)
                 "current temperature cold  level secondary side"    annotation (Placement(
              transformation(
              extent={{-20,-20},{20,20}},
              rotation=0,
              origin={-80,20})));
        Real T_prim_hot(
          unit="K",
          displayUnit="degC",
          min=277)
                 "current temperature hot level primary side"        annotation (Placement(
              transformation(
              extent={{-20,-20},{20,20}},
              rotation=0,
              origin={-80,140})));
        Real T_prim_cold(
          unit="K",
          displayUnit="degC",
          min=277)
                  "current temperature cold level primary side"       annotation (Placement(
              transformation(
              extent={{-20,-20},{20,20}},
              rotation=0,
              origin={-80,100})));
        Modelica.Blocks.Interfaces.RealInput Q_dot_set(unit="kW", displayUnit="kW")
          "setpoint heat transfer (positive production, negative consumption)"
          annotation (Placement(transformation(extent={{-20,-20},{20,20}},
              rotation=-90,
              origin={-60,188})));
        Real V_dot_prim(unit="l/min", displayUnit="l/min")
          annotation (Placement(transformation(extent={{-100,-40},{-60,0}})));
        Real V_dot_sec(unit="l/min", displayUnit="l/min")
          annotation (Placement(transformation(extent={{-100,-80},{-60,-40}})));

        Real Q_dot_is(unit="kW", displayUnit="kW")
          "currently transferred heat (positive production, negative consumption)"
          annotation (Placement(transformation(extent={{-100,-120},{-60,-80}})));
        Modelica.Blocks.Continuous.LimPID PID_prim_cons(
          controllerType=controllerType,
          k=k_prim_cons,
          Ti=Ti_prim_cons,
          Td=Td_prim_cons,
          yMax=1,
          yMin=0.25,
          initType=initType,
          y_start=PID_prim_cons.yMax)
          annotation (Placement(transformation(extent={{-30,-32},{-10,-12}})));
        Modelica.Blocks.Continuous.LimPID PID_sec_cons(
          controllerType=controllerType,
          k=k_sec_cons,
          Ti=Ti_sec_cons,
          Td=Td_sec_cons,
          yMax=V_dot_sec_max,
          yMin=2,
          initType=initType,
          y_start=PID_sec_cons.yMax)
          annotation (Placement(transformation(extent={{-34,28},{-14,48}})));
        Modelica.Blocks.Continuous.LimPID PID_prim_prod(
          controllerType=controllerType,
          k=k_prim_prod,
          Ti=Ti_prim_prod,
          Td=Td_prim_prod,
          yMax=1,
          yMin=0.25,
          initType=initType,
          y_start=PID_prim_prod.yMax) annotation (Placement(transformation(extent={{12,-30},
                  {32,-10}})));
        Modelica.Blocks.Continuous.LimPID PID_sec_prod(
          controllerType=controllerType,
          k=k_sec_prod,
          Ti=Ti_sec_prod,
          Td=Td_sec_prod,
          yMax=V_dot_sec_max,
          yMin=2,
          initType=initType,
          y_start=PID_sec_prod.yMax) annotation (Placement(transformation(extent={{10,28},
                  {30,48}})));
        Real T_sec_set(unit="K", displayUnit="degC")
           "Temperature on the secondary side" annotation (Placement(transformation(
              extent={{-20,-20},{20,20}},
              rotation=0,
              origin={80,100})));
        Real V_dot_sec_set(unit="l/min", displayUnit=
             "l/min") "volume flow rate setpoint on the secondary side" annotation (Placement(transformation(
              extent={{-20,-20},{20,20}},
              rotation=0,
              origin={80,60})));
        Modelica.Blocks.Interfaces.RealInput T_sec_in_is(unit="K", displayUnit="degC")
          "Temperature on the secondary side" annotation (Placement(transformation(
              extent={{-20,-20},{20,20}},
              rotation=270,
              origin={60,190})));

      equation

        // assign inputs
        T_prim_hot   = states[1];
        T_prim_cold  = states[2];
        T_sec_hot    = states[3];
        T_sec_cold   = states[4];
        V_dot_prim   = states[5];
        V_dot_sec    = states[6];
        Q_dot_is     = states[7];
        Delta_p_prim = states[8];

        Delta_T_prim      = T_prim_hot -T_prim_cold;
        Delta_T_sec       = T_sec_hot  -T_sec_cold;

        beta_prim_prod = 1 - alpha_prim_prod;
        beta_sec_prod  = 1 - alpha_sec_prod;
        beta_prim_cons = 1 - alpha_prim_cons;
        beta_sec_cons  = 1 - alpha_sec_cons;

        // determine easy static values that just depend on prosumer mode
        // determine inputs for the four PIDs
        // four PIDs in order to be able to have different gains for each situation

        if  Q_dot_set <= 0-tol then // consumption mode
          prosumer_mode = -1;
        elseif Q_dot_set >= 0+tol then // production mode
          prosumer_mode = +1;
        else // idle mode
          prosumer_mode = 0;
        end if;

        Q_dot_is_use = abs(Q_dot_is);
        Q_dot_set_use = abs(Q_dot_set);

        if prosumer_mode == -1 then // consumption mode
          pi_set = 1;
          mu_set = -1;
          T_prim_relev_des = DeltaT_prim_des;
          T_prim_relev_is = T_prim_hot-T_prim_cold;
          T_sec_relev_des = T_sec_hot_des;
          T_sec_relev_is = T_sec_hot;

          PIDin_prim_cons_is_weighted    = alpha_prim_cons*Q_dot_is_use/Delta_Qdot_norm + beta_prim_cons*(-1)*T_prim_relev_is/Delta_T_norm;
          PIDin_prim_cons_des_weighted   = alpha_prim_cons*Q_dot_set_use/Delta_Qdot_norm + beta_prim_cons*(-1)*T_prim_relev_des/Delta_T_norm;
          PIDin_prim_prod_is_weighted    = 0;
          PIDin_prim_prod_des_weighted   = 0;
          PIDin_sec_cons_is_weighted     = alpha_sec_cons*Q_dot_is_use/Delta_Qdot_norm + beta_sec_cons*(-1)*T_sec_relev_is/Delta_T_norm;
          PIDin_sec_cons_des_weighted    = alpha_sec_cons*Q_dot_set_use/Delta_Qdot_norm + beta_sec_cons*(-1)*T_sec_relev_des/Delta_T_norm;
          PIDin_sec_prod_is_weighted     = 0;
          PIDin_sec_prod_des_weighted    = 0;

          error_prim_weighted            = PIDin_prim_cons_des_weighted - PIDin_prim_cons_is_weighted;
          error_sec_weighted             = PIDin_sec_cons_des_weighted - PIDin_sec_cons_is_weighted;

          error_T_prim_abs               = DeltaT_prim_des - Delta_T_prim;
          error_T_sec_abs                = T_sec_hot_des - T_sec_hot;

          error_T_high_prio_abs          = T_sec_hot_des - T_sec_hot;
          error_T_low_prio_abs           = DeltaT_prim_des - Delta_T_prim;

        elseif prosumer_mode == 1 then // production mode
          pi_set = 1;
          mu_set = 1;
          T_prim_relev_des = DeltaT_sec_des;
          T_prim_relev_is = T_sec_hot-T_sec_cold;
          T_sec_relev_des = T_prim_hot_des;
          T_sec_relev_is = T_prim_hot;

          PIDin_prim_cons_is_weighted    = 0;
          PIDin_prim_cons_des_weighted   = 0;
          PIDin_prim_prod_is_weighted    = alpha_prim_prod*Q_dot_is_use/Delta_Qdot_norm + beta_prim_prod*T_prim_relev_is/Delta_T_norm;
          PIDin_prim_prod_des_weighted   = alpha_prim_prod*Q_dot_set_use/Delta_Qdot_norm + beta_prim_prod*T_prim_relev_des/Delta_T_norm;
          PIDin_sec_cons_is_weighted     = 0;
          PIDin_sec_cons_des_weighted    = 0;
          PIDin_sec_prod_is_weighted     = alpha_sec_prod*Q_dot_is_use/Delta_Qdot_norm + beta_sec_prod*T_sec_relev_is/Delta_T_norm;
          PIDin_sec_prod_des_weighted    = alpha_sec_prod*Q_dot_set_use/Delta_Qdot_norm + beta_sec_prod*T_sec_relev_des/Delta_T_norm;

          error_prim_weighted            = PIDin_prim_prod_des_weighted - PIDin_prim_prod_is_weighted;
          error_sec_weighted             = PIDin_sec_prod_des_weighted - PIDin_sec_prod_is_weighted;

          error_T_prim_abs               = T_prim_hot_des - T_prim_hot;
          error_T_sec_abs                = DeltaT_sec_des - Delta_T_sec;

          error_T_high_prio_abs          = T_prim_hot_des - T_prim_hot; // T_pim_hot
          error_T_low_prio_abs           = DeltaT_sec_des - Delta_T_sec; // Deltat_T_sec

        else // idle mode
          pi_set = 0;
          mu_set = -1;
          T_prim_relev_des = 0;
          T_prim_relev_is = 0;
          T_sec_relev_des = 0;
          T_sec_relev_is = 0;

          PIDin_prim_cons_is_weighted    = 0;
          PIDin_prim_cons_des_weighted   = 0;
          PIDin_prim_prod_is_weighted    = 0;
          PIDin_prim_prod_des_weighted   = 0;
          PIDin_sec_cons_is_weighted     = 0;
          PIDin_sec_cons_des_weighted    = 0;
          PIDin_sec_prod_is_weighted     = 0;
          PIDin_sec_prod_des_weighted    = 0;

          error_prim_weighted            = 0;
          error_sec_weighted             = 0;

          error_T_prim_abs               = 0;
          error_T_sec_abs                = 0;

          error_T_high_prio_abs          = 0;
          error_T_low_prio_abs           = 0;

        end if;

        // assign PID controller inputs
        PID_prim_cons.u_s    = PIDin_prim_cons_des_weighted;
        PID_prim_cons.u_m    = PIDin_prim_cons_is_weighted;
        PID_prim_prod.u_s    = PIDin_prim_prod_des_weighted;
        PID_prim_prod.u_m    = PIDin_prim_prod_is_weighted;
        PID_sec_cons.u_s     = PIDin_sec_cons_des_weighted;
        PID_sec_cons.u_m     = PIDin_sec_cons_is_weighted;
        PID_sec_prod.u_s     = PIDin_sec_prod_des_weighted;
        PID_sec_prod.u_m     = PIDin_sec_prod_is_weighted;

        // connect secondary side temperature setpoint
        T_sec_set    =  T_sec_in_is;

        // assign PID outputs to controller outputs
        if prosumer_mode == -1 then // consumption mode
          u_set = 0;
          kappa_set =PID_prim_cons.y;
          V_dot_sec_set = PID_sec_cons.y;
        elseif prosumer_mode == 1 then // production mode
          u_set =PID_prim_prod.y;
          kappa_set = 0;
          V_dot_sec_set = PID_sec_prod.y;
        else // idle mode
          V_dot_sec_set = 0;
          u_set = 0;
          kappa_set = 0;
        end if;

        error_Q_abs = Q_dot_set_use - Q_dot_is_use;

        // assign control variables vector
        contr_vars_real[1]   =  T_sec_set;
        contr_vars_real[2]   =  V_dot_sec_set;
        contr_vars_real[3]   =  pi_set;
        contr_vars_real[4]   =  mu_set;
        contr_vars_real[5]   =  u_set;
        contr_vars_real[6]   =  kappa_set;

          annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-120,-160},{
                  120,180}}),                                           graphics={
                Text(
                extent={{-70,56},{64,-56}},
                textColor={28,108,200},
                textString="weighted
PID"),       Rectangle(extent={{-120,180},{120,-160}}, lineColor={0,0,0})}),
                                                                         Diagram(
              coordinateSystem(preserveAspectRatio=false, extent={{-120,-160},{120,180}})));
      end PID_Q_T_weighted_mix2;

      package auxiliary
        block TimeTable_noInterp
          "Generate a discontinuous and non-interpolated signal from a table by using the forward value."

          parameter Real table[:, 2] = fill(0.0, 0, 2)
            "Table matrix (time = first column; e.g., table=[0, 0; 1, 1; 2, 4])"
            annotation(Dialog(groupImage="modelica://Modelica/Resources/Images/Blocks/Sources/TimeTable.png"));
          parameter Modelica.Units.SI.Time timeScale(min=Modelica.Constants.eps)=1
            "Time scale of first table column" annotation (Evaluate=true);
          extends Modelica.Blocks.Interfaces.SignalSource;
          parameter Modelica.Units.SI.Time shiftTime=startTime
            "Shift time of first table column";
        protected
          discrete Real a "Interpolation coefficient a of actual interval (y=a*x+b)";
          discrete Real b "Interpolation coefficient b of actual interval (y=a*x+b)";
          Integer last(start=1) "Last used lower grid index";
          discrete Modelica.Units.SI.Time nextEvent(start=0, fixed=true)
            "Next event instant";
          discrete Real nextEventScaled(start=0, fixed=true)
            "Next scaled event instant";
          Real timeScaled "Scaled time";

          function getInterpolationCoefficients
            "Determine interpolation coefficients and next time event"
            extends Modelica.Icons.Function;
            input Real table[:, 2] "Table for interpolation";
            input Real offset "y-offset";
            input Real startTimeScaled "Scaled time-offset";
            input Real timeScaled "Actual scaled time instant";
            input Integer last "Last used lower grid index";
            input Real TimeEps "Relative epsilon to check for identical time instants";
            input Real shiftTimeScaled "Time shift";
            output Real a "Interpolation coefficient a (y=a*x + b)";
            output Real b "Interpolation coefficient b (y=a*x + b)";
            output Real nextEventScaled "Next scaled event instant";
            output Integer next "New lower grid index";
          protected
            Integer columns=2 "Column to be interpolated";
            Integer ncol=2 "Number of columns to be interpolated";
            Integer nrow=size(table, 1) "Number of table rows";
            Integer next0;
            Real tp;
            Real dt;
          algorithm
            next := last;
            nextEventScaled := timeScaled - TimeEps*abs(timeScaled);
            // in case there are no more time events
            tp := timeScaled + TimeEps*abs(timeScaled);

            if tp < startTimeScaled then
              // First event not yet reached
              nextEventScaled := startTimeScaled;
              a := 0;
              b := offset;
            elseif nrow < 2 then
              // Special action if table has only one row
              a := 0;
              b := offset + table[1, columns];
            else
              tp := tp - shiftTimeScaled;
              // Find next time event instant. Note, that two consecutive time instants
              // in the table may be identical due to a discontinuous point.
              while next < nrow and tp >= table[next, 1] loop
                next := next + 1;
              end while;

              // Define next time event, if last table entry not reached
              if next < nrow then
                nextEventScaled := shiftTimeScaled + table[next, 1];
              end if;

              // Determine interpolation coefficients
              if next == 1 then
                next := 2;
              end if;
              next0 := next - 1;
              dt := table[next, 1] - table[next0, 1];
              if dt <= TimeEps*abs(table[next, 1]) then
                // Interpolation interval is not big enough, use "next" value
                a := 0;
                b := offset + table[next, columns];
              else
                a := table[next0, columns]; // (table[next, columns] - table[next0, columns])/dt;
                b := 0; // offset + table[next0, columns] - a*table[next0, 1];
              end if;
            end if;
            // Take into account shiftTimeScaled "a*(time - shiftTime) + b"
            b := b - a*shiftTimeScaled;
          end getInterpolationCoefficients;
        algorithm
          if noEvent(size(table, 1) > 1) then
            assert(not (table[1, 1] > 0.0 or table[1, 1] < 0.0), "The first point in time has to be set to 0, but is table[1,1] = " + String(table[1, 1]));
          end if;
          when {time >= pre(nextEvent),initial()} then
            (a,b,nextEventScaled,last) := getInterpolationCoefficients(
                table,
                offset,
                startTime/timeScale,
                timeScaled,
                last,
                100*Modelica.Constants.eps,
                shiftTime/timeScale);
            nextEvent := nextEventScaled*timeScale;
          end when;
        equation
          assert(size(table, 1) > 0, "No table values defined.");
          timeScaled = time/timeScale;
          y = a; // a*timeScaled + b;
          annotation (
            Icon(coordinateSystem(
                preserveAspectRatio=true,
                extent={{-100,-100},{100,100}}), graphics={
                Line(points={{-80,68},{-80,-80}}, color={192,192,192}),
                Polygon(
                  points={{-80,90},{-88,68},{-72,68},{-80,90}},
                  lineColor={192,192,192},
                  fillColor={192,192,192},
                  fillPattern=FillPattern.Solid),
                Line(points={{-90,-70},{82,-70}}, color={192,192,192}),
                Polygon(
                  points={{90,-70},{68,-62},{68,-78},{90,-70}},
                  lineColor={192,192,192},
                  fillColor={192,192,192},
                  fillPattern=FillPattern.Solid),
                Rectangle(
                  extent={{-48,70},{2,-50}},
                  lineColor={255,255,255},
                  fillColor={192,192,192},
                  fillPattern=FillPattern.Solid),
                Line(points={{-48,-50},{-48,70},{52,70},{52,-50},{-48,-50},{-48,-20},
                      {52,-20},{52,10},{-48,10},{-48,40},{52,40},{52,70},{2,70},{2,-51}}),
                Text(
                  extent={{-150,-150},{150,-110}},
                  textString="offset=%offset")}),
                Documentation(info="<html>
<p>
This block generates an output signal by <strong>linear interpolation</strong> in
a table. The time points and function values are stored in a matrix
<strong>table[i,j]</strong>, where the first column table[:,1] contains the
time points and the second column contains the data to be interpolated.
The table interpolation has the following properties:
</p>
<ul>
<li>The interpolation interval is found by a linear search where the interval used in the
    last call is used as start interval.</li>
<li>The time points need to be <strong>monotonically increasing</strong>.</li>
<li><strong>Discontinuities</strong> are allowed, by providing the same
    time point twice in the table.</li>
<li>Values <strong>outside</strong> of the table range, are computed by
    <strong>extrapolation</strong> through the last or first two points of the
    table.</li>
<li>If the table has only <strong>one row</strong>, no interpolation is performed and
    the function value is just returned independently of the actual time instant.</li>
<li>Via parameters <strong>shiftTime</strong> and <strong>offset</strong> the curve defined
    by the table can be shifted both in time and in the ordinate value.
    The time instants stored in the table are therefore <strong>relative</strong>
    to <strong>shiftTime</strong>.</li>
<li>If time &lt; startTime, no interpolation is performed and the offset
    is used as ordinate value for the output.</li>
<li>If the table has more than one row, the first point in time <strong>always</strong> has to be set to <strong>0</strong>, e.g.,
    <strong>table=[1,1;2,2]</strong> is <strong>illegal</strong>. If you want to
    shift the time table in time use the <strong>shiftTime</strong> parameter instead.</li>
<li>The table is implemented in a numerically sound way by
    generating <strong>time events</strong> at interval boundaries.
    This generates continuously differentiable values for the integrator.</li>
<li>Via parameter <strong>timeScale</strong> the first column of the table array can
    be scaled, e.g., if the table array is given in hours (instead of seconds)
    <strong>timeScale</strong> shall be set to 3600.</li>
</ul>
<p>
Example:
</p>
<blockquote><pre>
   table = [0, 0;
            1, 0;
            1, 1;
            2, 4;
            3, 9;
            4, 16];
If, e.g., time = 1.0, the output y =  0.0 (before event), 1.0 (after event)
    e.g., time = 1.5, the output y =  2.5,
    e.g., time = 2.0, the output y =  4.0,
    e.g., time = 5.0, the output y = 23.0 (i.e., extrapolation).
</pre></blockquote>

<p>
<img src=\"modelica://Modelica/Resources/Images/Blocks/Sources/TimeTable.png\"
     alt=\"TimeTable.png\">
</p>

</html>",       revisions="<html>
<h4>Release Notes</h4>
<ul>
<li><em>Oct. 21, 2002</em>
       by Christian Schweiger:<br>
       Corrected interface from
<blockquote><pre>
parameter Real table[:, :]=[0, 0; 1, 1; 2, 4];
</pre></blockquote>
       to
<blockquote><pre>
parameter Real table[:, <strong>2</strong>]=[0, 0; 1, 1; 2, 4];
</pre></blockquote>
       </li>
<li><em>Nov. 7, 1999</em>
       by <a href=\"http://www.robotic.dlr.de/Martin.Otter/\">Martin Otter</a>:<br>
       Realized.</li>
</ul>
</html>"));
        end TimeTable_noInterp;

        model test_procedure

          parameter Real dotQ_low( min = 0) = 8
          "heat transfer setpoint for high heat transfer in test procedure";

          parameter Real dotQ_high( min= 0) = 12
          "heat transfer setpoint for low heat transfer in test procedure";

          parameter Real Tsecin_warm(  min= 0) = 45
          "warm inlet temperature for secondary side";

          parameter Real Tsecin_cold(  min= 0) = 30
           "cold inlet temperature for secondary side";

          parameter Real noise_mu( min = 0) = 0
          "mean value of noise for temperature"
          annotation(Dialog(group="Noise"));

          parameter Real noise_sigma( min = 0) = 5
          "standard deviation of noise for temperature"
          annotation(Dialog(group="Noise"));

          parameter Integer test_procedure[:,3] = [
                0,0,0;
                900,1,6;
                1800,1,6;
                2700,1,6;
                3600,0,0;
                4500,0,0;
                5400,0,0;
                6300,1,6;
                7200,1,9;
                8100,1,6;
                9000,0,0;
                9900,-1,6;
                10800,-1,9;
                11700,-1,6;
                12600,0,0;
                13500,1,9;
                14400,0,0;
                15300,0,0;
                16200,-1,9;
                17100,0,0;
                18000,0,0]
          "time [s], prosumer mode (producer[1], consumer[-1], idle[0]), heat transfer (high[9], low[6])";

          Modelica.Blocks.Sources.CombiTimeTable combiTimeTable(
            tableOnFile=false,
            table=test_procedure,
            smoothness=Modelica.Blocks.Types.Smoothness.ConstantSegments,
            extrapolation=Modelica.Blocks.Types.Extrapolation.Periodic)
            "time [s], prosumer mode (producer[1], consumer[-1], idle[0]), heat transfer (high[9], low[6])"
            annotation (Placement(transformation(extent={{-16,2},{4,22}})));

          Modelica.Blocks.Noise.NormalNoise normalNoise(
            samplePeriod=30,
            mu=noise_mu,
            sigma=noise_sigma)
            annotation (Placement(transformation(extent={{-62,-46},{-42,-26}})));

          Modelica.Blocks.Interfaces.RealOutput dotQ
            annotation (Placement(transformation(extent={{90,30},{110,50}})));
          Modelica.Blocks.Interfaces.RealOutput T
            annotation (Placement(transformation(extent={{90,-30},{110,-10}})));
          inner Modelica.Blocks.Noise.GlobalSeed globalSeed(enableNoise=false,
              fixedSeed=4345)
            annotation (Placement(transformation(extent={{-80,64},{-60,84}})));
        equation

          if combiTimeTable.y[1] == 1 then
            T = Tsecin_warm + 273.15 + normalNoise.y;
              if combiTimeTable.y[2] == 6 then
                dotQ = dotQ_low;
              elseif combiTimeTable.y[2] == 9 then
                dotQ = dotQ_high;
              else
                dotQ = 0;
              end if;
          elseif combiTimeTable.y[1] == -1 then
            T = Tsecin_cold + 273.15 + normalNoise.y;
              if combiTimeTable.y[2] == 6 then
                dotQ = -dotQ_low;
              elseif combiTimeTable.y[2] == 9 then
                dotQ = -dotQ_high;
              else
                dotQ = 0;
              end if;
          else
            T = Tsecin_cold + 273.15;
            dotQ = 0;
          end if;

          annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
                coordinateSystem(preserveAspectRatio=false)));
        end test_procedure;

        function simpleMATLAB_fileConverter
        "Function to import trajectory result files and write them as MatLab compatible .mat f
iles"
        input String filename="filename" "File to be converted" annotation (Dialog(__Dymola_loadSelector(filter="Matlab files (*.mat)",
        caption="Select the results trajectory file")));
        input String varOrigNames[:]={"Time","J1.w","J2.w"} "Variable names/headers in the fil
e in modelica syntax";
        input String varReNames[:]={"Time","Inertia_1_angularVel","Inertia_2_angularVel"}
        "Variable names which will appear in the MATLAB results file";
        input String outputFilename="outputFile.mat";
        protected
        Integer noRows "Number of rows in the trajectory being converted";
        Integer noColumn=12 "Number of columns in the trajectory being converted";
        Real data[:,:] "Data read in from trajectory file";
        Real dataDump[:,:] "Sacrificial dump variable for writeMatrix command";
        Integer i=2 "Loop counter";
        algorithm
        noRows := DymolaCommands.Trajectories.readTrajectorySize(filename);
        data := DymolaCommands.Trajectories.readTrajectory(
        filename,
        varOrigNames,
        noRows);
        data := transpose(data);
        noColumn := size(data, 2);
        while i <= noColumn loop
        dataDump := [data[:, 1],data[:, i]];
        if i == 2 then
        DymolaCommands.MatrixIO.writeMatrix(
        outputFilename,
        varReNames[i],
        dataDump);
        else
        DymolaCommands.MatrixIO.writeMatrix(
        outputFilename,
        varReNames[i],
        dataDump,
        true);
        end if;
        i := i + 1;
        end while;
        annotation (Documentation(info="<html>
<p></p>
</html>"),         uses(DymolaCommands(version="1.4")));
        end simpleMATLAB_fileConverter;
      end auxiliary;
    end Controller_PID_based;

    package Tests "Testing the new models and especially controllers"

      model Test_heat_exchanger

        replaceable package Medium1 = ProsNet.Media.Water;
        replaceable package Medium2 = ProsNet.Media.Water;

        extends ProsNet.Prosumers.BaseClasses.PrimarySideParameters;
        extends ProsNet.Prosumers.SecondarySides.BaseClasses.PumpsPairDynParam;
        extends
          ProsNet.Prosumers.SecondarySides.BaseClasses.ControlVolumeDynParam;

        inner Modelica.Fluid.System system
          annotation (Placement(transformation(extent={{-30,54},{-10,74}})));
        Modelica.Fluid.Sources.MassFlowSource_T boundary(
          redeclare package Medium = ProsNet.Media.Water,
          m_flow=0.1,
          T=333.15,
          nPorts=1) annotation (Placement(transformation(extent={{-84,14},{-64,34}})));
        Modelica.Fluid.Sources.FixedBoundary boundary1(redeclare package Medium =
              ProsNet.Media.Water,
                           nPorts=1)
          annotation (Placement(transformation(extent={{-88,-90},{-68,-70}})));
        Modelica.Fluid.Sources.MassFlowSource_T boundary2(
          redeclare package Medium = ProsNet.Media.Water,
          use_m_flow_in=true,
          m_flow=1,
          T=313.15,
          nPorts=1) annotation (Placement(transformation(extent={{70,-34},{50,-14}})));
        Modelica.Fluid.Sources.FixedBoundary boundary3(redeclare package Medium =
              ProsNet.Media.Water,
                           nPorts=1)
          annotation (Placement(transformation(extent={{52,44},{32,64}})));
        Modelica.Blocks.Sources.Ramp ramp(
          height=0.1,
          duration=900,
          offset=0.1,
          startTime=900)
          annotation (Placement(transformation(extent={{50,-70},{70,-50}})));

        ProsNet.Fluid.HeatExchangers.LiquidToLiquid liquidToLiquid(
          redeclare package Medium1 = Medium1,
          redeclare package Medium2 = Medium2,
          m1_flow_nominal=1,
          m2_flow_nominal=1,
          dp1_nominal(displayUnit="bar") = 100000,
          dp2_nominal(displayUnit="bar") = 100000,
          use_Q_flow_nominal=true,
          Q_flow_nominal(displayUnit="kW") = 30000,
          T_a1_nominal=313.15,
          T_a2_nominal=333.15,
          eps_nominal=1)
          annotation (Placement(transformation(extent={{-24,-18},{-4,2}})));
        ProsNet.Fluid.Sensors.Temperature senTem2(redeclare package Medium =
              Medium1)
          annotation (Placement(transformation(extent={{46,12},{66,32}})));
        ProsNet.Fluid.Sensors.Temperature senTem1(redeclare package Medium =
              Medium2)
          annotation (Placement(transformation(extent={{-46,-70},{-26,-50}})));

        Real DeltaT1;
        Real DeltaT2;
      equation
        connect(ramp.y, boundary2.m_flow_in) annotation (Line(points={{71,-60},{76,-60},{76,
                -16},{70,-16}},   color={0,0,127}));

        DeltaT2 =senTem2.T - liquidToLiquid.T_in2;
        DeltaT1 = senTem1.T - liquidToLiquid.T_in1;

        connect(boundary.ports[1], liquidToLiquid.port_a1)
          annotation (Line(points={{-64,24},{-62,24},{-62,-2},{-24,-2}}, color={0,127,255}));
        connect(boundary1.ports[1], liquidToLiquid.port_b2) annotation (Line(points={{-68,-80},
                {-50,-80},{-50,-54},{-48,-54},{-48,-14},{-24,-14}}, color={0,127,255}));
        connect(liquidToLiquid.port_a2, boundary2.ports[1]) annotation (Line(points={{-4,-14},
                {2,-14},{2,-18},{0,-18},{0,-24},{50,-24}}, color={0,127,255}));
        connect(liquidToLiquid.port_b1, boundary3.ports[1])
          annotation (Line(points={{-4,-2},{26,-2},{26,54},{32,54}}, color={0,127,255}));
        connect(senTem1.port, liquidToLiquid.port_b2) annotation (Line(points={{-36,-70},{-36,
                -74},{-50,-74},{-50,-54},{-48,-54},{-48,-14},{-24,-14}}, color={0,127,255}));
        connect(senTem2.port, liquidToLiquid.port_b1)
          annotation (Line(points={{56,12},{56,-2},{-4,-2}}, color={0,127,255}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
              coordinateSystem(preserveAspectRatio=false), graphics={Line(
                points={{10,-34},{10,12}},
                color={0,255,0},
                thickness=1), Line(
                points={{10,10},{14,4}},
                color={0,255,0},
                thickness=1)}),
          experiment(
            StopTime=2100,
            Interval=1,
            __Dymola_Algorithm="Dassl"));
      end Test_heat_exchanger;

      model Test_pipe_model
        Modelica.Fluid.Sensors.VolumeFlowRate volumeFlowRate(redeclare package
            Medium =
              ProsNet.Media.Water)
          annotation (Placement(transformation(extent={{-20,32},{0,52}})));
        Modelica.Fluid.Sensors.RelativePressure relativePressure(redeclare
            package Medium =
                     ProsNet.Media.Water)
          annotation (Placement(transformation(extent={{20,64},{40,84}})));
        Modelica.Fluid.Sensors.RelativeTemperature relativeTemperature(redeclare
            package Medium = ProsNet.Media.Water)
          annotation (Placement(transformation(extent={{20,4},{40,-16}})));
        Modelica.Fluid.Sources.MassFlowSource_T mass_source(
          redeclare package Medium = ProsNet.Media.Water,
          use_m_flow_in=true,
          use_T_in=true,
          m_flow=1,
          T(displayUnit="K"),
          nPorts=1) annotation (Placement(transformation(extent={{-94,32},{-74,52}})));
        inner Modelica.Fluid.System system(
          T_ambient=285.15,
          allowFlowReversal=true,
          energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyStateInitial)
          annotation (Placement(transformation(extent={{132,76},{152,96}})));
        Modelica.Fluid.Sources.Boundary_pT bou(redeclare package Medium =
              ProsNet.Media.Water, nPorts=1)
          annotation (Placement(transformation(extent={{104,32},{84,52}})));
        ProsNet.BidirectionalSubstation.Substation_idealProsumer.Controller_PID_based.auxiliary.TimeTable_noInterp
          volume_flow(table=[0,1; 10,5; 20,10; 30,11.4; 40,15; 50,20; 60,1; 70,
              5; 80,10; 90,11.4; 100,15; 110,20; 120,1; 130,5; 140,10; 150,11.4;
              160,15; 170,20; 7470,20; 7480,0; 10980,20; 17580,20], timeScale=1)
          annotation (Placement(transformation(extent={{-182,50},{-162,70}})));
        ProsNet.BidirectionalSubstation.Substation_idealProsumer.Controller_PID_based.auxiliary.TimeTable_noInterp
          temperatures(
          table=[0,30; 10,30; 20,30; 30,30; 40,30; 50,30; 60,60; 70,60; 80,60;
              90,60; 100,60; 110,60; 120,90; 130,90; 140,90; 150,90; 160,90;
              170,90; 180,90; 7480,90; 10980,90; 17580,90],
          timeScale=1,
          y(unit="K"))
          annotation (Placement(transformation(extent={{-182,16},{-162,36}})));
        Modelica.Blocks.Math.Gain gain(k=1/60.266)
          annotation (Placement(transformation(extent={{-138,50},{-118,70}})));
        Modelica.Blocks.Math.UnitConversions.From_degC from_degC
          annotation (Placement(transformation(extent={{-138,16},{-118,36}})));
        ProsNet.Fluid.Sensors.TemperatureTwoPort Tem_in(
          redeclare package Medium = ProsNet.Media.Water,
          m_flow_nominal=1/6,
          tau=0) annotation (Placement(transformation(extent={{-54,32},{-34,52}})));
        ProsNet.Fluid.Sensors.TemperatureTwoPort Tem_out(
          redeclare package Medium = ProsNet.Media.Water,
          m_flow_nominal=1/6,
          tau=0) annotation (Placement(transformation(extent={{54,32},{74,52}})));

        Real DeltaT;
        Modelica.Fluid.Sensors.VolumeFlowRate volumeFlowRate1(redeclare package
            Medium = ProsNet.Media.Water)
          annotation (Placement(transformation(extent={{-18,-80},{2,-60}})));
        Modelica.Fluid.Sensors.RelativePressure relativePressure1(redeclare
            package Medium =
                     ProsNet.Media.Water)
          annotation (Placement(transformation(extent={{22,-48},{42,-28}})));
        Modelica.Fluid.Sensors.RelativeTemperature relativeTemperature1(redeclare
            package Medium = ProsNet.Media.Water)
          annotation (Placement(transformation(extent={{22,-108},{42,-128}})));
        Modelica.Fluid.Sources.Boundary_pT bou1(redeclare package Medium =
              ProsNet.Media.Water, nPorts=1)
          annotation (Placement(transformation(extent={{106,-80},{86,-60}})));
        ProsNet.Fluid.Sensors.TemperatureTwoPort Tem_in1(
          redeclare package Medium = ProsNet.Media.Water,
          m_flow_nominal=1/6,
          tau=0) annotation (Placement(transformation(extent={{-52,-80},{-32,-60}})));
        ProsNet.Fluid.Sensors.TemperatureTwoPort Tem_out1(
          redeclare package Medium = ProsNet.Media.Water,
          m_flow_nominal=1/6,
          tau=0) annotation (Placement(transformation(extent={{56,-80},{76,-60}})));
        ProsNet.Fluid.Pipes.InsulatedPipe_plug pipe_new(
          allowFlowReversal=true,
          length=1000,
          energyDynamics=Modelica.Fluid.Types.Dynamics.DynamicFreeInitial)
          annotation (Placement(transformation(extent={{18,-80},{38,-60}})));
        Modelica.Fluid.Sources.MassFlowSource_T mass_source1(
          redeclare package Medium = ProsNet.Media.Water,
          use_m_flow_in=true,
          use_T_in=true,
          m_flow=1,
          T(displayUnit="K"),
          nPorts=1) annotation (Placement(transformation(extent={{-88,-80},{-68,-60}})));
        ProsNet.Fluid.Pipes.InsulatedPipe pipe(
          allowFlowReversal=true,
          length=1000,
          energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyStateInitial)
          annotation (Placement(transformation(extent={{18,30},{38,50}})));
      equation
        connect(volume_flow.y, gain.u) annotation (Line(
            points={{-161,60},{-161,60},{-142,60},{-140,60}},
            color={0,0,127},
            smooth=Smooth.Bezier));
        connect(temperatures.y, from_degC.u) annotation (Line(
            points={{-161,26},{-161,26},{-144,26},{-140,26}},
            color={0,0,127},
            smooth=Smooth.Bezier));
        connect(mass_source.ports[1], Tem_in.port_a) annotation (Line(
            points={{-74,42},{-54,42}},
            color={0,127,255},
            smooth=Smooth.Bezier));
        connect(Tem_in.port_b, volumeFlowRate.port_a) annotation (Line(
            points={{-34,42},{-20,42}},
            color={0,127,255},
            smooth=Smooth.Bezier));
        connect(Tem_out.port_b, bou.ports[1]) annotation (Line(
            points={{74,42},{84,42}},
            color={0,127,255},
            smooth=Smooth.Bezier));

        DeltaT = Tem_out.T - Tem_in.T;
        connect(volumeFlowRate.port_b, relativePressure.port_a) annotation (Line(
              points={{0,42},{8,42},{8,74},{20,74}}, color={0,127,255}));
        connect(relativePressure.port_b, Tem_out.port_a) annotation (Line(points={{40,
                74},{48,74},{48,42},{54,42}}, color={0,127,255}));
        connect(relativeTemperature.port_b, Tem_out.port_a) annotation (Line(points={
                {40,-6},{46,-6},{46,42},{54,42}}, color={0,127,255}));
        connect(relativeTemperature.port_a, volumeFlowRate.port_b) annotation (Line(
              points={{20,-6},{10,-6},{10,42},{0,42}}, color={0,127,255}));
        connect(Tem_in1.port_b, volumeFlowRate1.port_a) annotation (Line(
            points={{-32,-70},{-18,-70}},
            color={0,127,255},
            smooth=Smooth.Bezier));
        connect(Tem_out1.port_b, bou1.ports[1]) annotation (Line(
            points={{76,-70},{86,-70}},
            color={0,127,255},
            smooth=Smooth.Bezier));
        connect(volumeFlowRate1.port_b, relativePressure1.port_a) annotation (Line(
              points={{2,-70},{10,-70},{10,-38},{22,-38}}, color={0,127,255}));
        connect(relativePressure1.port_b, Tem_out1.port_a) annotation (Line(points={{
                42,-38},{50,-38},{50,-70},{56,-70}}, color={0,127,255}));
        connect(relativeTemperature1.port_b, Tem_out1.port_a) annotation (Line(points=
               {{42,-118},{50,-118},{50,-70},{56,-70}}, color={0,127,255}));
        connect(relativeTemperature1.port_a, volumeFlowRate1.port_b) annotation (Line(
              points={{22,-118},{10,-118},{10,-70},{2,-70}}, color={0,127,255}));
        connect(volumeFlowRate1.port_b, pipe_new.port_a)
          annotation (Line(points={{2,-70},{18,-70}}, color={0,127,255}));
        connect(pipe_new.port_b, Tem_out1.port_a)
          annotation (Line(points={{38,-70},{56,-70}}, color={0,127,255}));
        connect(mass_source1.ports[1], Tem_in1.port_a)
          annotation (Line(points={{-68,-70},{-52,-70}}, color={0,127,255}));
        connect(gain.y, mass_source.m_flow_in)
          annotation (Line(points={{-117,60},{-94,60},{-94,50}}, color={0,0,127}));
        connect(gain.y, mass_source1.m_flow_in) annotation (Line(points={{-117,60},{
                -104,60},{-104,-56},{-88,-56},{-88,-62}}, color={0,0,127}));
        connect(from_degC.y, mass_source.T_in) annotation (Line(points={{-117,26},{
                -102,26},{-102,46},{-96,46}}, color={0,0,127}));
        connect(from_degC.y, mass_source1.T_in) annotation (Line(points={{-117,26},{
                -106,26},{-106,-66},{-90,-66}}, color={0,0,127}));
        connect(pipe.port_a, relativePressure.port_a) annotation (Line(points={{18,40},
                {12,40},{12,42},{8,42},{8,74},{20,74}}, color={0,127,255}));
        connect(pipe.port_b, Tem_out.port_a) annotation (Line(points={{38,40},{42,40},
                {42,42},{54,42}}, color={0,127,255}));
        annotation (                                                       experiment(
            StopTime=17500,
            Interval=0.1,
            __Dymola_Algorithm="Dassl"));
      end Test_pipe_model;

      model Test_pump_curve

        ProsNet.Fluid.Pumps.SpeedControlled_y
                                      pump_prim_prod(
          redeclare final package Medium = ProsNet.Media.Water,
          final energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyStateInitial,
          inputType=ProsNet.Fluid.Types.InputType.Continuous,
          final tau=1,
          redeclare final ProsNet.Fluid.Pumps.Data.Pumps.IMP.NMTPlus152025to40 per,
          final use_inputFilter=true,
          final riseTime=1,
          final init=Modelica.Blocks.Types.Init.SteadyState,
          final y_start=0)                annotation (Placement(transformation(
              extent={{10,10},{-10,-10}},
              rotation=-90,
              origin={-44,-8})));
        ProsNet.Fluid.Sources.Boundary_pT
                                  bou(redeclare package Medium =
              ProsNet.Media.Water,
            nPorts=1)
          annotation (Placement(transformation(extent={{68,36},{48,56}})));
        Modelica.Blocks.Sources.RealExpression realExpression(y=1)
          annotation (Placement(transformation(extent={{-168,2},{-148,22}})));
        Modelica.Blocks.Sources.Ramp ramp(
          height=-1,
          duration=1200,
          offset=1,
          startTime=5)
          annotation (Placement(transformation(extent={{140,-42},{120,-22}})));
        Modelica.Fluid.Valves.ValveLinear valveLinear(
          redeclare package Medium = ProsNet.Media.Water,
          dp_start=0,
          dp_nominal=70000,
          m_flow_nominal=0.55)
          annotation (Placement(transformation(extent={{42,-20},{62,0}})));
      equation
        connect(realExpression.y, pump_prim_prod.y) annotation (Line(points={{-147,12},
                {-64,12},{-64,-8},{-56,-8}}, color={0,0,127}));
        connect(pump_prim_prod.port_b, bou.ports[1]) annotation (Line(points={{-44,2},
                {-44,20},{42,20},{42,46},{48,46}}, color={0,127,255}));
        connect(pump_prim_prod.port_b, valveLinear.port_a) annotation (Line(points={{-44,2},
                {-44,20},{42,20},{42,2},{32,2},{32,-10},{42,-10}},        color={0,
                127,255}));
        connect(valveLinear.port_b, pump_prim_prod.port_a) annotation (Line(points={{62,-10},
                {64,-10},{64,-26},{-44,-26},{-44,-18}},         color={0,127,255}));
        connect(ramp.y, valveLinear.opening) annotation (Line(points={{119,-32},{66,
                -32},{66,2},{52,2},{52,-2}}, color={0,0,127}));
        annotation ();
      end Test_pump_curve;

      model Test_check_valve_model
        Modelica.Fluid.Sensors.VolumeFlowRate volumeFlowRate(redeclare package
            Medium =
              ProsNet.Media.Water)
          annotation (Placement(transformation(extent={{-20,32},{0,52}})));
        Modelica.Fluid.Sensors.RelativePressure relativePressure(redeclare
            package Medium =
                     ProsNet.Media.Water)
          annotation (Placement(transformation(extent={{20,64},{40,84}})));
        Modelica.Fluid.Sources.MassFlowSource_T mass_source(
          redeclare package Medium = ProsNet.Media.Water,
          use_m_flow_in=true,
          use_T_in=true,
          m_flow=1,
          T(displayUnit="K"),
          nPorts=1) annotation (Placement(transformation(extent={{-94,32},{-74,52}})));
        inner Modelica.Fluid.System system(
          T_ambient=285.15,
          allowFlowReversal=true,
          energyDynamics=Modelica.Fluid.Types.Dynamics.DynamicFreeInitial)
          annotation (Placement(transformation(extent={{64,-52},{84,-32}})));
        Modelica.Fluid.Sources.Boundary_pT bou(redeclare package Medium =
              ProsNet.Media.Water,
          T=309.9,                 nPorts=1)
          annotation (Placement(transformation(extent={{104,32},{84,52}})));
        ProsNet.BidirectionalSubstation.Substation_idealProsumer.Controller_PID_based.auxiliary.TimeTable_noInterp
          mass_flow_kg_s(table=[0,0; 900,0.1; 1800,0.2; 2700,0.3; 3600,0.4;
              4500,0.5; 5400,0.6; 6300,0.7; 7200,0.8; 8100,0.9; 9000,1; 9900,
              1.1; 10800,1.2; 11700,1.3; 12600,1.4; 13500,1.5; 14400,1.6; 15300,
              1.7; 16200,1.8; 17100,1.9; 18000,2], timeScale=1)
          annotation (Placement(transformation(extent={{-182,50},{-162,70}})));
        ProsNet.BidirectionalSubstation.Substation_idealProsumer.Controller_PID_based.auxiliary.TimeTable_noInterp
          temperatures(
          table=[0,36.75; 900,36.75; 1800,36.75; 2700,36.75; 3600,36.75; 4500,
              36.75; 5400,36.75; 6300,36.75; 7200,36.75; 8100,36.75; 9000,36.75;
              9900,36.75; 10800,36.75; 11700,36.75; 12600,36.75; 13500,36.75;
              14400,36.75; 15300,36.75; 16200,36.75; 17100,36.75; 18000,36.75],
          timeScale=1,
          y(unit="K"))
          annotation (Placement(transformation(extent={{-182,16},{-162,36}})));

        Modelica.Blocks.Math.UnitConversions.From_degC from_degC
          annotation (Placement(transformation(extent={{-138,16},{-118,36}})));
        ProsNet.Fluid.Sensors.TemperatureTwoPort Tem_in(
          redeclare package Medium = ProsNet.Media.Water,
          m_flow_nominal=1/6,
          tau=1) annotation (Placement(transformation(extent={{-54,32},{-34,52}})));
        ProsNet.Fluid.Sensors.TemperatureTwoPort Tem_out(
          redeclare package Medium = ProsNet.Media.Water,
          m_flow_nominal=1/6,
          tau=1) annotation (Placement(transformation(extent={{54,32},{74,52}})));

        Real DeltaT;
        Modelica.Blocks.Sources.Constant const(k=1)
          annotation (Placement(transformation(extent={{-20,66},{0,86}})));
        ProsNet.Fluid.FixedResistances.CheckValve cheVal_prim_cons(
          m_flow_nominal=1.25,
          redeclare final package Medium = ProsNet.Media.Water,
          final CvData=ProsNet.Fluid.Types.CvTypes.Kv,
          final Kv=6.29,
          final l=0.002) annotation (Placement(transformation(
              extent={{10,-10},{-10,10}},
              rotation=180,
              origin={32,42})));
      equation
        connect(temperatures.y, from_degC.u) annotation (Line(
            points={{-161,26},{-161,26},{-144,26},{-140,26}},
            color={0,0,127},
            smooth=Smooth.Bezier));
        connect(from_degC.y, mass_source.T_in) annotation (Line(
            points={{-117,26},{-102,26},{-102,46},{-96,46}},
            color={0,0,127},
            smooth=Smooth.Bezier));
        connect(mass_source.ports[1], Tem_in.port_a) annotation (Line(
            points={{-74,42},{-54,42}},
            color={0,127,255},
            smooth=Smooth.Bezier));
        connect(Tem_in.port_b, volumeFlowRate.port_a) annotation (Line(
            points={{-34,42},{-20,42}},
            color={0,127,255},
            smooth=Smooth.Bezier));
        connect(Tem_out.port_b, bou.ports[1]) annotation (Line(
            points={{74,42},{84,42}},
            color={0,127,255},
            smooth=Smooth.Bezier));

        DeltaT = Tem_out.T - Tem_in.T;
        connect(mass_flow_kg_s.y, mass_source.m_flow_in) annotation (Line(points=
                {{-161,60},{-128,60},{-128,50},{-94,50}}, color={0,0,127}));
        connect(cheVal_prim_cons.port_b, Tem_out.port_a)
          annotation (Line(points={{42,42},{54,42}}, color={0,127,255}));
        connect(volumeFlowRate.port_b, cheVal_prim_cons.port_a)
          annotation (Line(points={{0,42},{22,42}}, color={0,127,255}));
        connect(relativePressure.port_a, cheVal_prim_cons.port_a)
          annotation (Line(points={{20,74},{22,74},{22,42}}, color={0,127,255}));
        connect(relativePressure.port_b, cheVal_prim_cons.port_b) annotation (
            Line(points={{40,74},{42,74},{42,42},{42,42}}, color={0,127,255}));
        annotation (                                                       experiment(
            StopTime=18000,
            Interval=10,
            __Dymola_Algorithm="Dassl"));
      end Test_check_valve_model;

      model Test_control_valve_model
        Modelica.Fluid.Sensors.VolumeFlowRate volumeFlowRate(redeclare package
            Medium =
              ProsNet.Media.Water)
          annotation (Placement(transformation(extent={{-20,32},{0,52}})));
        Modelica.Fluid.Sensors.RelativePressure relativePressure(redeclare
            package Medium =
                     ProsNet.Media.Water)
          annotation (Placement(transformation(extent={{20,64},{40,84}})));
        Modelica.Fluid.Sources.MassFlowSource_T mass_source(
          redeclare package Medium = ProsNet.Media.Water,
          use_m_flow_in=true,
          use_T_in=true,
          m_flow=1,
          T(displayUnit="K"),
          nPorts=1) annotation (Placement(transformation(extent={{-94,32},{-74,52}})));
        inner Modelica.Fluid.System system(
          T_ambient=285.15,
          allowFlowReversal=true,
          energyDynamics=Modelica.Fluid.Types.Dynamics.DynamicFreeInitial)
          annotation (Placement(transformation(extent={{64,-52},{84,-32}})));
        Modelica.Fluid.Sources.Boundary_pT bou(redeclare package Medium =
              ProsNet.Media.Water,
          T=309.9,                 nPorts=1)
          annotation (Placement(transformation(extent={{104,32},{84,52}})));
        ProsNet.BidirectionalSubstation.Substation_idealProsumer.Controller_PID_based.auxiliary.TimeTable_noInterp
          mass_flow_kg_s(table=[0,0; 900,0.1; 1800,0.2; 2700,0.3; 3600,0.4;
              4500,0.5; 5400,0.6; 6300,0.7; 7200,0.8; 8100,0.9; 9000,1; 9900,
              1.1; 10800,1.2; 11700,1.3; 12600,1.4; 13500,1.5; 14400,1.6; 15300,
              1.7; 16200,1.8; 17100,1.9; 18000,2], timeScale=1)
          annotation (Placement(transformation(extent={{-182,50},{-162,70}})));
        ProsNet.BidirectionalSubstation.Substation_idealProsumer.Controller_PID_based.auxiliary.TimeTable_noInterp
          temperatures(
          table=[0,36.75; 900,36.75; 1800,36.75; 2700,36.75; 3600,36.75; 4500,
              36.75; 5400,36.75; 6300,36.75; 7200,36.75; 8100,36.75; 9000,36.75;
              9900,36.75; 10800,36.75; 11700,36.75; 12600,36.75; 13500,36.75;
              14400,36.75; 15300,36.75; 16200,36.75; 17100,36.75; 18000,36.75],
          timeScale=1,
          y(unit="K"))
          annotation (Placement(transformation(extent={{-182,16},{-162,36}})));

        Modelica.Blocks.Math.UnitConversions.From_degC from_degC
          annotation (Placement(transformation(extent={{-138,16},{-118,36}})));
        ProsNet.Fluid.Sensors.TemperatureTwoPort Tem_in(
          redeclare package Medium = ProsNet.Media.Water,
          m_flow_nominal=1/6,
          tau=1) annotation (Placement(transformation(extent={{-54,32},{-34,52}})));
        ProsNet.Fluid.Sensors.TemperatureTwoPort Tem_out(
          redeclare package Medium = ProsNet.Media.Water,
          m_flow_nominal=1/6,
          tau=1) annotation (Placement(transformation(extent={{54,32},{74,52}})));

        Real DeltaT;
        ProsNet.Fluid.Valves.TwoWayEqualPercentage valve_prim_cons(
          m_flow_nominal=1.25,
          kFixed=0,
          redeclare final package Medium = ProsNet.Media.Water,
          final CvData=ProsNet.Fluid.Types.CvTypes.Kv,
          final Kv=6.29,
          final use_inputFilter=true,
          final riseTime=35,
          final init=Modelica.Blocks.Types.Init.InitialState,
          final y_start=0,
          final l=0.002) annotation (Placement(transformation(
              extent={{-10,10},{10,-10}},
              rotation=180,
              origin={26,34})));
        Modelica.Blocks.Sources.Constant const(k=1)
          annotation (Placement(transformation(extent={{-20,66},{0,86}})));
      equation
        connect(temperatures.y, from_degC.u) annotation (Line(
            points={{-161,26},{-161,26},{-144,26},{-140,26}},
            color={0,0,127},
            smooth=Smooth.Bezier));
        connect(from_degC.y, mass_source.T_in) annotation (Line(
            points={{-117,26},{-102,26},{-102,46},{-96,46}},
            color={0,0,127},
            smooth=Smooth.Bezier));
        connect(mass_source.ports[1], Tem_in.port_a) annotation (Line(
            points={{-74,42},{-54,42}},
            color={0,127,255},
            smooth=Smooth.Bezier));
        connect(Tem_in.port_b, volumeFlowRate.port_a) annotation (Line(
            points={{-34,42},{-20,42}},
            color={0,127,255},
            smooth=Smooth.Bezier));
        connect(Tem_out.port_b, bou.ports[1]) annotation (Line(
            points={{74,42},{84,42}},
            color={0,127,255},
            smooth=Smooth.Bezier));

        DeltaT = Tem_out.T - Tem_in.T;
        connect(mass_flow_kg_s.y, mass_source.m_flow_in) annotation (Line(points=
                {{-161,60},{-128,60},{-128,50},{-94,50}}, color={0,0,127}));
        connect(const.y, valve_prim_cons.y) annotation (Line(points={{1,76},{14,
                76},{14,46},{26,46}}, color={0,0,127}));
        connect(volumeFlowRate.port_b, valve_prim_cons.port_b) annotation (Line(
              points={{0,42},{10,42},{10,34},{16,34}}, color={0,127,255}));
        connect(valve_prim_cons.port_a, Tem_out.port_a) annotation (Line(points={
                {36,34},{46,34},{46,42},{54,42}}, color={0,127,255}));
        connect(relativePressure.port_b, valve_prim_cons.port_a)
          annotation (Line(points={{40,74},{40,34},{36,34}}, color={0,127,255}));
        connect(valve_prim_cons.port_b, relativePressure.port_a) annotation (Line(
              points={{16,34},{6,34},{6,74},{20,74}}, color={0,127,255}));
        annotation (                                                       experiment(
            StopTime=18000,
            Interval=10,
            __Dymola_Algorithm="Dassl"));
      end Test_control_valve_model;

      model Test_TwoProsumers
        new_prosumer_models.heat_transfer_station B1(n=0.5, redeclare
            ProsNet.Fluid.Pumps.Data.Pumps.IMP.NMTSmart25_120to180
                                                           feedinPer)
                                                            annotation (Placement(
              transformation(
              extent={{20,-18},{-20,18}},
              rotation=0,
              origin={-48,8})));
        Controller_PID_based.PID_Q_T_weighted_sameside Ctrl1(alpha_prim_prod=0.35,
            alpha_sec_cons=0.35) annotation (Placement(transformation(
              extent={{-12,-17},{12,17}},
              rotation=0,
              origin={-44,73})));
        Controller_PID_based.auxiliary.TimeTable_noInterp power_set1(table=[0,10; 900,10;
              1800,10; 2700,10; 3600,-10; 4500,-4; 5400,4; 6300,4])  annotation (Placement(
              transformation(
              extent={{-10,-10},{10,10}},
              rotation=-90,
              origin={-70,134})));
        Controller_PID_based.auxiliary.TimeTable_noInterp temp_sec_in1(table=[0,55; 900,55;
              1800,55; 2700,55; 3600,30; 4500,30; 5400,55; 6300,55])   annotation (
            Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=-90,
              origin={-28,134})));
        ProsNet.Fluid.Pipes.InsulatedPipe_plug pipe_hot12
          annotation (Placement(transformation(extent={{-8,-58},{18,-32}})));
        new_prosumer_models.heat_transfer_station B2(n=0.5, redeclare
            ProsNet.Fluid.Pumps.Data.Pumps.IMP.NMTSmart25_120to180
                                                           feedinPer)
                                                            annotation (Placement(
              transformation(
              extent={{20,-18},{-20,18}},
              rotation=0,
              origin={50,8})));
        Controller_PID_based.PID_Q_T_weighted_sameside Ctrl2(alpha_prim_prod=0.35,
            alpha_sec_cons=0.35) annotation (Placement(transformation(
              extent={{-12,-17},{12,17}},
              rotation=0,
              origin={52,73})));
        Controller_PID_based.auxiliary.TimeTable_noInterp power_set2(table=[0,-10; 900,-10;
              1800,-10; 2700,-10; 3600,10; 4500,4; 5400,-4; 6300,-4])
                                                                     annotation (Placement(
              transformation(
              extent={{-10,-10},{10,10}},
              rotation=-90,
              origin={28,134})));
        Controller_PID_based.auxiliary.TimeTable_noInterp temp_sec_in2(table=[0,30; 900,30;
              1800,30; 2700,30; 3600,55; 4500,55; 5400,30; 6300,30])   annotation (
            Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=-90,
              origin={70,134})));
        ProsNet.Fluid.Pipes.InsulatedPipe_plug pipe_cold12
          annotation (Placement(transformation(extent={{18,-103},{-8,-77}})));
        Modelica.Fluid.Sources.Boundary_pT boundary(
          redeclare package Medium = ProsNet.Media.Water,
          use_p_in=false,
          T=325.4,
          nPorts=1) annotation (Placement(transformation(extent={{-92,-55},{-72,-35}})));
        inner Modelica.Fluid.System system(T_ambient=285.15)
          annotation (Placement(transformation(extent={{-92,-114},{-72,-94}})));
        Modelica.Blocks.Math.Add add annotation (Placement(transformation(
              extent={{-5,-5},{5,5}},
              rotation=-90,
              origin={-31,103})));
        Modelica.Blocks.Sources.RealExpression realExpression(y=273.15) annotation (Placement(
              transformation(
              extent={{-5,-6},{5,6}},
              rotation=270,
              origin={-8,117})));
        Modelica.Blocks.Sources.RealExpression realExpression1(y=273.15) annotation (
            Placement(transformation(
              extent={{-5,-6},{5,6}},
              rotation=270,
              origin={82,115})));
        Modelica.Blocks.Math.Add add1 annotation (Placement(transformation(
              extent={{-5,-5},{5,5}},
              rotation=-90,
              origin={59,101})));
      equation
        connect(B1.contr_vars_real, Ctrl1.contr_vars_real)
          annotation (Line(points={{-27.8,8},{-20,8},{-20,72},{-32,72}}, color={0,0,127}));
        connect(Ctrl1.states, B1.states)
          annotation (Line(points={{-56,72},{-74,72},{-74,8},{-68,8}}, color={0,0,127}));
        connect(power_set1.y, Ctrl1.Q_dot_set) annotation (Line(points={{-70,123},{-70,118},{
                -50,118},{-50,90.8}},  color={0,0,127}));
        connect(B2.contr_vars_real,Ctrl2. contr_vars_real)
          annotation (Line(points={{70.2,8},{78,8},{78,72},{64,72}},     color={0,0,127}));
        connect(Ctrl2.states,B2. states)
          annotation (Line(points={{40,72},{24,72},{24,8},{30,8}},     color={0,0,127}));
        connect(power_set2.y,Ctrl2. Q_dot_set) annotation (Line(points={{28,123},{28,118},{
                46,118},{46,90.8}},    color={0,0,127}));
        connect(B1.hot_prim, pipe_hot12.port_a)
          annotation (Line(points={{-34,-10.2},{-34,-45},{-8,-45}}, color={0,127,255}));
        connect(pipe_hot12.port_b, B2.hot_prim)
          annotation (Line(points={{18,-45},{64,-45},{64,-10.2}}, color={0,127,255}));
        connect(B1.cold_prim, pipe_cold12.port_b)
          annotation (Line(points={{-62,-10},{-62,-90},{-8,-90}}, color={0,127,255}));
        connect(pipe_cold12.port_a, B2.cold_prim)
          annotation (Line(points={{18,-90},{36,-90},{36,-10}}, color={0,127,255}));
        connect(pipe_hot12.port_a, boundary.ports[1])
          annotation (Line(points={{-8,-45},{-72,-45}}, color={0,127,255}));
        connect(add.y, Ctrl1.T_sec_in_is)
          annotation (Line(points={{-31,97.5},{-31,91},{-38,91}}, color={0,0,127}));
        connect(temp_sec_in1.y, add.u2) annotation (Line(points={{-28,123},{-28,114},{-34,114},
                {-34,109}}, color={0,0,127}));
        connect(add.u1, realExpression.y) annotation (Line(points={{-28,109},{-12,109},{-12,
                106},{-8,106},{-8,111.5}}, color={0,0,127}));
        connect(temp_sec_in2.y, add1.u2)
          annotation (Line(points={{70,123},{70,114},{56,114},{56,107}}, color={0,0,127}));
        connect(add1.y, Ctrl2.T_sec_in_is)
          annotation (Line(points={{59,95.5},{58,95.5},{58,91}}, color={0,0,127}));
        connect(realExpression1.y, add1.u1) annotation (Line(points={{82,109.5},{82,104},{68,
                104},{68,107},{62,107}}, color={0,0,127}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-120},{
                  200,160}})),                                         Diagram(
              coordinateSystem(preserveAspectRatio=false, extent={{-100,-120},{200,160}}),
                                                           graphics={Rectangle(extent={{-92,
                    154},{-2,-20}}, lineColor={28,108,200}),         Rectangle(extent={{6,
                    154},{96,-20}}, lineColor={28,108,200})}),
          experiment(
            StopTime=6300,
            Interval=0.1,
            __Dymola_Algorithm="Dassl"));
      end Test_TwoProsumers;
    end Tests;

  end Substation_idealProsumer;

  package Controller_PID_based

    model PID_Q_T_weighted_crossover_new

      import Modelica.Units.SI;
      import T_AbsZeroDegC = Modelica.Constants.T_zero;
      import Modelica.Blocks.Types.Init;
      import Modelica.Blocks.Types.SimpleController;

        // !!!!! parameters !!!!!
      parameter Real Delta_Qdot_norm = 1
          "Heat power value for normalizing the error (deviation) of the transferred heat.
      For alpha=0.5 a deviation of Delta_Qdot_norm in heat transfer is weigthed equal to a deviation of Delta_T_norm in temperature."
          annotation(Dialog(group="Normalizing values"));
      parameter SI.TemperatureDifference Delta_T_norm(min=0) = 3
          "Temperature difference for normalizing the error (deviation) of the temperature.
      For alpha=0.5 a deviation of Delta_T_norm in temperature is weighted equal to a deviation of Delta_Qdot_norm in heat transfer."
          annotation(Dialog(group="Normalizing values"));

      parameter SI.Temperature T_prim_hot_des(min=277)= - T_AbsZeroDegC + 53.5
          "desired temperature supply primary side"
          annotation(Dialog(group="Temperature objectives"));
      parameter SI.Temperature T_sec_hot_des(min=277)= - T_AbsZeroDegC + 50
          "desired temperature supply secondary side"
          annotation(Dialog(group="Temperature objectives"));
      parameter SI.TemperatureDifference DeltaT_prim_des(min=1) =   20
          "desired temperature difference primary side"
          annotation(Dialog(group="Temperature objectives"));
      parameter SI.TemperatureDifference DeltaT_sec_des(min=1) =   20
          "desired temperature difference secondary side"
          annotation(Dialog(group="Temperature objectives"));
      parameter Real u_set_sec_prod_max = 1
        "maximum secondary side volume flow in [l/min]"
        annotation(Dialog(group="General PID settings"));
      parameter Real k_prim_prod = 1.5
        "Proportional gain for controller in [-]"
        annotation(Dialog(group="PID primary side - producer mode - tuning"));
      parameter Real Ti_prim_prod = 8
        "Integral time constant for controller in [s]"
        annotation(Dialog(group="PID primary side - producer mode - tuning"));
      parameter Real Td_prim_prod = 0
        "Derivative time constant for controller in [s]"
        annotation(Dialog(group="PID primary side - producer mode - tuning"));
      parameter Real alpha_prim_prod(min=0, max=1) = 0.666
        "weight for the relevance of the error of the transferred heat in comparison to the error of temperature objectives (sum is one)"
        annotation(Dialog(group="PID primary side - producer mode - tuning"));
      parameter Real k_sec_prod = 1.5
        "Proportional gain for controller in [-]"
        annotation(Dialog(group="PID secondary side - producer mode - tuning"));
      parameter Real Ti_sec_prod = 8
        "Integral time constant for controller in [s]"
        annotation(Dialog(group="PID secondary side - producer mode - tuning"));
      parameter Real Td_sec_prod = 0
        "Derivative time constant for controller in [s]"
        annotation(Dialog(group="PID secondary side - producer mode - tuning"));
      parameter Real alpha_sec_prod(min=0, max=1) = 0.333
        "weight for the relevance of the error of the transferred heat in comparison to the error of temperature objectives (sum is one)"
        annotation(Dialog(group="PID secondary side - producer mode - tuning"));
      parameter Real k_prim_cons = 1
        "Proportional gain for controller in [-]"
        annotation(Dialog(group="PID primary side - consumer mode - tuning"));
      parameter Real Ti_prim_cons = 35
        "Integral time constant for controller in [s]"
        annotation(Dialog(group="PID primary side - consumer mode - tuning"));
      parameter Real Td_prim_cons = 0
        "Derivative time constant for controller in [s]"
        annotation(Dialog(group="PID primary side - consumer mode - tuning"));
      parameter Real alpha_prim_cons(min=0, max=1) = 0.333
        "weight for the relevance of the error of the transferred heat in comparison to the error of temperature objectives (sum is one)"
        annotation(Dialog(group="PID primary side - consumer mode - tuning"));
      parameter Real k_sec_cons = 1.5
        "Proportional gain for controller in [-]"
        annotation(Dialog(group="PID secondary side - consumer mode - tuning"));
      parameter Real Ti_sec_cons = 8
        "Integral time constant for controller in [s]"
        annotation(Dialog(group="PID secondary side - consumer mode - tuning"));
      parameter Real Td_sec_cons = 0
        "Derivative time constant for controller in [s]"
        annotation(Dialog(group="PID secondary side - consumer mode - tuning"));
      parameter Real alpha_sec_cons(min=0, max=1) = 0.666
        "weight for the relevance of the error of the transferred heat in comparison to the error of temperature objectives (sum is one)"
        annotation(Dialog(group="PID secondary side - consumer mode - tuning"));
      parameter .Modelica.Blocks.Types.SimpleController controllerType=
             Modelica.Blocks.Types.SimpleController.PID "Type of controller"
             annotation(Dialog(tab="Advanced", group="PIDs general"));
      parameter Init initType = Modelica.Blocks.Types.Init.NoInit
        "Type of initialization (1: no init, 2: steady state, 3: initial state, 4: initial output)"
        annotation(Dialog(tab="Advanced", group="PIDs general"));
      parameter Real tol = 0.1
        "tolerance [kW] for idle mode concerning heat transfer setpoint dotQ"
        annotation(Dialog(tab="Advanced", group="Miscellaneous"));

      // !!!!! variables !!!!!
      Real beta_prim_prod(min=0, max=1)
        "weight for the relevance of the error of the temperature(difference)";
      Real beta_sec_prod(min=0, max=1)
        "weight for the relevance of the error of the temperature(difference)";
      Real beta_prim_cons(min=0, max=1)
        "weight for the relevance of the error of the temperature(difference)";
      Real beta_sec_cons(min=0, max=1)
        "weight for the relevance of the error of the temperature(difference)";
      Integer  prosumer_mode
        "prosumer mode {-1;0;1}";
      Real T_prim_relev_des
          "desired value of relevant temperature (difference)
      for control of primary side";
      Real T_prim_relev_is
          "current value of relevant temperature (difference)
      for control of primary side";
      Real T_sec_relev_des
          "desired value of relevant temperature (difference)
      for control of secondary side";
      Real T_sec_relev_is
          "current value of relevant temperature (difference)
      for control of secondary side";

      Real PIDin_prim_cons_is_weighted
          "weighted input of is-values for PID_prim_cons";
      Real PIDin_prim_cons_des_weighted
          "weighted input of desired values for PID_prim_cons";
      Real PIDin_prim_prod_is_weighted
          "weighted input of is-values for PID_prim_prod";
      Real PIDin_prim_prod_des_weighted
          "weighted input of desired values for PID_prim_prod";
      Real PIDin_sec_cons_is_weighted
          "weighted input of is-values for PID_sec_cons";
      Real PIDin_sec_cons_des_weighted
          "weighted input of desired values for PID_sec_cons";
      Real PIDin_sec_prod_is_weighted
          "weighted input of is-values for PID_sec_prod";
      Real PIDin_sec_prod_des_weighted
          "weighted input of desired values for PID_sec_prod";

      Real error_prim_weighted
          "weighted overall error of primary side controller";

      Real error_sec_weighted
          "weighted overall error of primary side controller";

      Real error_T_prim_abs
          "temperature error of primary side controller";

      Real error_T_sec_abs
          "temperature error of primary side controller";

      Real error_Q_abs
          "temperature error of primary side controller";

      Real error_T_high_prio_abs
          "error of higher prioritized temperature objective";

      Real error_T_low_prio_abs
          "error of lower prioritized temperature objective";

      Real Delta_T_prim
          "weighted overall error of primary side controller";

      Real Delta_T_sec
          "weighted overall error of primary side controller";

      Real Q_dot_is_use;
      Real Q_dot_set_use;

       Real Delta_p_prim(unit="Pa", displayUnit="bar") annotation (
          Placement(transformation(
            extent={{-20,-20},{20,20}},
            rotation=180,
            origin={-80,-140})));

      // !!!!! ports !!!!!

      Modelica.Blocks.Interfaces.RealVectorInput states[8]
        annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));

      Modelica.Blocks.Interfaces.RealVectorOutput contr_vars_real[6]
        annotation (Placement(transformation(extent={{100,-20},{140,20}})));

      Real u_set_prim
        "Normalized velocity of feed-in pump"
          annotation (Placement(transformation(
            extent={{-20,-20},{20,20}},
            rotation=0,
            origin={80,-60}),  iconTransformation(
            extent={{-20,-20},{20,20}},
            rotation=0,
            origin={100,-60})));
      Real kappa_set
        "Normalized flow coefficient for control valve"
         annotation (Placement(transformation(
            extent={{-20,-20},{20,20}},
            rotation=0,
            origin={80,-100}),iconTransformation(
            extent={{-20,-20},{20,20}},
            rotation=0,
            origin={100,-100})));
      Real pi_set
        "Participation" annotation (Placement(transformation(
            extent={{-20,-20},{20,20}},
            rotation=0,
            origin={80,20}),    iconTransformation(
            extent={{-20,-20},{20,20}},
            rotation=0,
            origin={100,20})));
      Real mu_set
        "Operating mode" annotation (Placement(transformation(
            extent={{-20,-20},{20,20}},
            rotation=0,
            origin={80,-20}),  iconTransformation(
            extent={{-20,-20},{20,20}},
            rotation=0,
            origin={100,-20})));
      Real T_sec_hot(
        unit="K",
        displayUnit="degC",
        min=277) "current temperature hot level secondary side"      annotation (Placement(
            transformation(
            extent={{-20,-20},{20,20}},
            rotation=0,
            origin={-80,60})));
      Real T_sec_cold(
        unit="K",
        displayUnit="degC",
        min=277)
               "current temperature cold  level secondary side"    annotation (Placement(
            transformation(
            extent={{-20,-20},{20,20}},
            rotation=0,
            origin={-80,20})));
      Real T_prim_hot(
        unit="K",
        displayUnit="degC",
        min=277)
               "current temperature hot level primary side"        annotation (Placement(
            transformation(
            extent={{-20,-20},{20,20}},
            rotation=0,
            origin={-80,140})));
      Real T_prim_cold(
        unit="K",
        displayUnit="degC",
        min=277)
                "current temperature cold level primary side"       annotation (Placement(
            transformation(
            extent={{-20,-20},{20,20}},
            rotation=0,
            origin={-80,100})));
      Modelica.Blocks.Interfaces.RealInput Q_dot_set(unit="kW", displayUnit="kW")
        "setpoint heat transfer (positive production, negative consumption)"
        annotation (Placement(transformation(extent={{-20,-20},{20,20}},
            rotation=-90,
            origin={-60,188})));
      Real V_dot_prim(unit="l/min", displayUnit="l/min")
        annotation (Placement(transformation(extent={{-100,-40},{-60,0}})));
      Real u2
        annotation (Placement(transformation(extent={{-100,-80},{-60,-40}})));

      Real Q_dot_is(unit="kW", displayUnit="kW")
        "currently transferred heat (positive production, negative consumption)"
        annotation (Placement(transformation(extent={{-100,-120},{-60,-80}})));
      Modelica.Blocks.Continuous.LimPID PID_prim_cons(
        controllerType=controllerType,
        k=k_prim_cons,
        Ti=Ti_prim_cons,
        Td=Td_prim_cons,
        yMax=1,
        yMin=0.25,
        initType=initType,
        y_start=PID_prim_cons.yMax)
        annotation (Placement(transformation(extent={{-30,-32},{-10,-12}})));
      Modelica.Blocks.Continuous.LimPID PID_sec_cons(
        controllerType=controllerType,
        k=k_sec_cons,
        Ti=Ti_sec_cons,
        Td=Td_sec_cons,
        yMax=1,
        yMin=0,
        initType=initType,
        y_start=PID_sec_cons.yMax)
        annotation (Placement(transformation(extent={{-34,28},{-14,48}})));
      Modelica.Blocks.Continuous.LimPID PID_prim_prod(
        controllerType=controllerType,
        k=k_prim_prod,
        Ti=Ti_prim_prod,
        Td=Td_prim_prod,
        yMax=1,
        yMin=0.25,
        initType=initType,
        y_start=PID_prim_prod.yMax) annotation (Placement(transformation(extent={{12,-30},
                {32,-10}})));
      Modelica.Blocks.Continuous.LimPID PID_sec_prod(
        controllerType=controllerType,
        k=k_sec_prod,
        Ti=Ti_sec_prod,
        Td=Td_sec_prod,
        yMax=1,
        yMin=0,
        initType=initType,
        y_start=PID_sec_prod.yMax) annotation (Placement(transformation(extent={{10,28},
                {30,48}})));
      Real u_set_sec_cons
         "Normalized velocity of the secondary side pump in production" annotation (Placement(transformation(
            extent={{-20,-20},{20,20}},
            rotation=0,
            origin={80,100})));
      Real u_set_sec_prod
       "speed set for secondary side pupm in consumption mode" annotation (Placement(transformation(
            extent={{-20,-20},{20,20}},
            rotation=0,
            origin={80,60})));
      Modelica.Blocks.Interfaces.RealInput T_sec_in_is(unit="K", displayUnit="degC")
        "Temperature on the secondary side" annotation (Placement(transformation(
            extent={{-20,-20},{20,20}},
            rotation=270,
            origin={60,190})));

    equation

      // assign inputs
      T_prim_hot   = states[1];
      T_prim_cold  = states[2];
      T_sec_hot    = states[3];
      T_sec_cold   = states[4];
      V_dot_prim   = states[5];
      u2           = states[6];
      Q_dot_is     = states[7];
      Delta_p_prim = states[8];

      Delta_T_prim      = T_prim_hot -T_prim_cold;
      Delta_T_sec       = T_sec_hot  -T_sec_cold;

      beta_prim_prod = 1 - alpha_prim_prod;
      beta_sec_prod  = 1 - alpha_sec_prod;
      beta_prim_cons = 1 - alpha_prim_cons;
      beta_sec_cons  = 1 - alpha_sec_cons;

      // determine easy static values that just depend on prosumer mode
      // determine inputs for the four PIDs
      // four PIDs in order to be able to have different gains for each situation

      if  Q_dot_set <= 0-tol then // consumption mode
        prosumer_mode = -1;
      elseif Q_dot_set >= 0+tol then // production mode
        prosumer_mode = +1;
      else // idle mode
        prosumer_mode = 0;
      end if;

      Q_dot_is_use = abs(Q_dot_is);
      Q_dot_set_use = abs(Q_dot_set);

      if prosumer_mode == -1 then // consumption mode
        pi_set = 1;
        mu_set = -1;
        T_prim_relev_des = T_sec_hot_des;
        T_prim_relev_is = T_sec_hot;
        T_sec_relev_des = DeltaT_prim_des;
        T_sec_relev_is = T_prim_hot-T_prim_cold;

        PIDin_prim_cons_is_weighted    = alpha_prim_cons*Q_dot_is_use/Delta_Qdot_norm + beta_prim_cons*T_prim_relev_is/Delta_T_norm;
        PIDin_prim_cons_des_weighted   = alpha_prim_cons*Q_dot_set_use/Delta_Qdot_norm + beta_prim_cons*T_prim_relev_des/Delta_T_norm;
        PIDin_prim_prod_is_weighted    = 0;
        PIDin_prim_prod_des_weighted   = 0;
        PIDin_sec_cons_is_weighted     = alpha_sec_cons*Q_dot_is_use/Delta_Qdot_norm + beta_sec_cons*T_sec_relev_is/Delta_T_norm;
        PIDin_sec_cons_des_weighted    = alpha_sec_cons*Q_dot_set_use/Delta_Qdot_norm +
          beta_sec_cons*T_sec_relev_des/Delta_T_norm;
        PIDin_sec_prod_is_weighted     = 0;
        PIDin_sec_prod_des_weighted    = 0;

        error_prim_weighted            = PIDin_prim_cons_des_weighted - PIDin_prim_cons_is_weighted;
        error_sec_weighted             = PIDin_sec_cons_des_weighted - PIDin_sec_cons_is_weighted;

        error_T_prim_abs               = DeltaT_prim_des - Delta_T_prim;
        error_T_sec_abs                = T_sec_hot_des - T_sec_hot;

        error_T_high_prio_abs          = T_sec_hot_des - T_sec_hot; // T_sec_hot
        error_T_low_prio_abs           = DeltaT_prim_des - Delta_T_prim; // Delta_T_prim

      elseif prosumer_mode == 1 then // production mode
        pi_set = 1;
        mu_set = 1;
        T_prim_relev_des = DeltaT_sec_des;
        T_prim_relev_is = T_sec_hot-T_sec_cold;
        T_sec_relev_des = T_prim_hot_des;
        T_sec_relev_is = T_prim_hot;

        PIDin_prim_cons_is_weighted    = 0;
        PIDin_prim_cons_des_weighted   = 0;
        PIDin_prim_prod_is_weighted    = alpha_prim_prod*Q_dot_is_use/Delta_Qdot_norm + beta_prim_prod*T_prim_relev_is/Delta_T_norm;
        PIDin_prim_prod_des_weighted   = alpha_prim_prod*Q_dot_set_use/Delta_Qdot_norm + beta_prim_prod*T_prim_relev_des/Delta_T_norm;
        PIDin_sec_cons_is_weighted     = 0;
        PIDin_sec_cons_des_weighted    = 0;
        PIDin_sec_prod_is_weighted     = alpha_sec_prod*Q_dot_is_use/Delta_Qdot_norm + beta_sec_prod*T_sec_relev_is/Delta_T_norm;
        PIDin_sec_prod_des_weighted    = alpha_sec_prod*Q_dot_set_use/Delta_Qdot_norm + beta_sec_prod*T_sec_relev_des/Delta_T_norm;

        error_prim_weighted            = PIDin_prim_prod_des_weighted - PIDin_prim_prod_is_weighted;
        error_sec_weighted             = PIDin_sec_prod_des_weighted - PIDin_sec_prod_is_weighted;

        error_T_prim_abs               = T_prim_hot_des - T_prim_hot;
        error_T_sec_abs                = DeltaT_sec_des - Delta_T_sec;

        error_T_high_prio_abs          = T_prim_hot_des - T_prim_hot; // T_pim_hot
        error_T_low_prio_abs           = DeltaT_sec_des - Delta_T_sec; // Deltat_T_sec

      else // idle mode
        pi_set = 0;
        mu_set = -1;
        T_prim_relev_des = 0;
        T_prim_relev_is = 0;
        T_sec_relev_des = 0;
        T_sec_relev_is = 0;

        PIDin_prim_cons_is_weighted    = 0;
        PIDin_prim_cons_des_weighted   = 0;
        PIDin_prim_prod_is_weighted    = 0;
        PIDin_prim_prod_des_weighted   = 0;
        PIDin_sec_cons_is_weighted     = 0;
        PIDin_sec_cons_des_weighted    = 0;
        PIDin_sec_prod_is_weighted     = 0;
        PIDin_sec_prod_des_weighted    = 0;

        error_prim_weighted            = 0;
        error_sec_weighted             = 0;

        error_T_prim_abs               = 0;
        error_T_sec_abs                = 0;

        error_T_high_prio_abs          = 0;
        error_T_low_prio_abs           = 0;

      end if;

      // assign PID controller inputs
      PID_prim_cons.u_s    = PIDin_prim_cons_des_weighted;
      PID_prim_cons.u_m    = PIDin_prim_cons_is_weighted;
      PID_prim_prod.u_s    = PIDin_prim_prod_des_weighted;
      PID_prim_prod.u_m    = PIDin_prim_prod_is_weighted;
      PID_sec_cons.u_s     = PIDin_sec_cons_des_weighted;
      PID_sec_cons.u_m     = PIDin_sec_cons_is_weighted;
      PID_sec_prod.u_s     = PIDin_sec_prod_des_weighted;
      PID_sec_prod.u_m     = PIDin_sec_prod_is_weighted;


      // assign PID outputs to controller outputs
      if prosumer_mode == -1 then // consumption mode
        u_set_prim = 0;
        kappa_set =PID_prim_cons.y;
        u_set_sec_cons = PID_sec_cons.y;
        u_set_sec_prod = 0;
      elseif prosumer_mode == 1 then // production mode
        u_set_prim =PID_prim_prod.y;
        kappa_set = 0;
        u_set_sec_cons = 0;
        u_set_sec_prod = 1;
      else // idle mode
        u_set_sec_cons = 0;
        u_set_sec_prod = 0;
        u_set_prim = 0;
        kappa_set = 0;
      end if;

      error_Q_abs = Q_dot_set_use - Q_dot_is_use;

      // assign control variables vector
      contr_vars_real[1]   =  u_set_sec_cons;
      contr_vars_real[2]   =  u_set_sec_prod;
      contr_vars_real[3]   =  pi_set;
      contr_vars_real[4]   =  mu_set;
      contr_vars_real[5]   =  u_set_prim;
      contr_vars_real[6]   =  kappa_set;

        annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-120,-160},{
                120,180}}),                                           graphics={
              Text(
              extent={{-70,56},{64,-56}},
              textColor={28,108,200},
              textString="weighted
PID"),     Rectangle(extent={{-120,180},{120,-160}}, lineColor={0,0,0})}),
                                                                       Diagram(
            coordinateSystem(preserveAspectRatio=false, extent={{-120,-160},{120,180}})));
    end PID_Q_T_weighted_crossover_new;

    model PID_Q_T_weighted_sameside

      import Modelica.Units.SI;
      import T_AbsZeroDegC = Modelica.Constants.T_zero;
      import Modelica.Blocks.Types.Init;
      import Modelica.Blocks.Types.SimpleController;

        // !!!!! parameters !!!!!
      parameter Real Delta_Qdot_norm = 1
          "Heat power value for normalizing the error (deviation) of the transferred heat.
      For alpha=0.5 a deviation of Delta_Qdot_norm in heat transfer is weigthed equal to a deviation of Delta_T_norm in temperature."
          annotation(Dialog(group="Normalizing values"));
      parameter SI.TemperatureDifference Delta_T_norm(min=0) = 3
          "Temperature difference for normalizing the error (deviation) of the temperature.
      For alpha=0.5 a deviation of Delta_T_norm in temperature is weighted equal to a deviation of Delta_Qdot_norm in heat transfer."
          annotation(Dialog(group="Normalizing values"));

      parameter SI.Temperature T_prim_hot_des(min=277)= - T_AbsZeroDegC + 53.5
          "desired temperature supply primary side"
          annotation(Dialog(group="Temperature objectives"));
      parameter SI.Temperature T_sec_hot_des(min=277)= - T_AbsZeroDegC + 50
          "desired temperature supply secondary side"
          annotation(Dialog(group="Temperature objectives"));
      parameter SI.TemperatureDifference DeltaT_prim_des(min=1) =   20
          "desired temperature difference primary side"
          annotation(Dialog(group="Temperature objectives"));
      parameter SI.TemperatureDifference DeltaT_sec_des(min=1) =   20
          "desired temperature difference secondary side"
          annotation(Dialog(group="Temperature objectives"));
      parameter Real V_dot_sec_max(unit="l/min", displayUnit="l/min") = 8.5
        "maximum secondary side volume flow in [l/min]"
        annotation(Dialog(group="General PID settings"));
      parameter Real k_prim_prod = 1.5
        "Proportional gain for controller in [-]"
        annotation(Dialog(group="PID primary side - producer mode - tuning"));
      parameter Real Ti_prim_prod = 8
        "Integral time constant for controller in [s]"
        annotation(Dialog(group="PID primary side - producer mode - tuning"));
      parameter Real Td_prim_prod = 0
        "Derivative time constant for controller in [s]"
        annotation(Dialog(group="PID primary side - producer mode - tuning"));
      parameter Real alpha_prim_prod(min=0, max=1) = 0.333
        "weight for the relevance of the error of the transferred heat in comparison to the error of temperature objectives (sum is one)"
        annotation(Dialog(group="PID primary side - producer mode - tuning"));
      parameter Real k_sec_prod = 1.5
        "Proportional gain for controller in [-]"
        annotation(Dialog(group="PID secondary side - producer mode - tuning"));
      parameter Real Ti_sec_prod = 8
        "Integral time constant for controller in [s]"
        annotation(Dialog(group="PID secondary side - producer mode - tuning"));
      parameter Real Td_sec_prod = 0
        "Derivative time constant for controller in [s]"
        annotation(Dialog(group="PID secondary side - producer mode - tuning"));
      parameter Real alpha_sec_prod(min=0, max=1) = 0.666
        "weight for the relevance of the error of the transferred heat in comparison to the error of temperature objectives (sum is one)"
        annotation(Dialog(group="PID secondary side - producer mode - tuning"));
      parameter Real k_prim_cons = 1.0
        "Proportional gain for controller in [-]"
        annotation(Dialog(group="PID primary side - consumer mode - tuning"));
      parameter Real Ti_prim_cons = 35
        "Integral time constant for controller in [s]"
        annotation(Dialog(group="PID primary side - consumer mode - tuning"));
      parameter Real Td_prim_cons = 0
        "Derivative time constant for controller in [s]"
        annotation(Dialog(group="PID primary side - consumer mode - tuning"));
      parameter Real alpha_prim_cons(min=0, max=1) = 0.666
        "weight for the relevance of the error of the transferred heat in comparison to the error of temperature objectives (sum is one)"
        annotation(Dialog(group="PID primary side - consumer mode - tuning"));
      parameter Real k_sec_cons = 1.5
        "Proportional gain for controller in [-]"
        annotation(Dialog(group="PID secondary side - consumer mode - tuning"));
      parameter Real Ti_sec_cons = 8
        "Integral time constant for controller in [s]"
        annotation(Dialog(group="PID secondary side - consumer mode - tuning"));
      parameter Real Td_sec_cons = 0
        "Derivative time constant for controller in [s]"
        annotation(Dialog(group="PID secondary side - consumer mode - tuning"));
      parameter Real alpha_sec_cons(min=0, max=1) = 0.333
        "weight for the relevance of the error of the transferred heat in comparison to the error of temperature objectives (sum is one)"
        annotation(Dialog(group="PID secondary side - consumer mode - tuning"));
      parameter .Modelica.Blocks.Types.SimpleController controllerType=
             Modelica.Blocks.Types.SimpleController.PID "Type of controller"
             annotation(Dialog(tab="Advanced", group="PIDs general"));
      parameter Init initType = Modelica.Blocks.Types.Init.NoInit
        "Type of initialization (1: no init, 2: steady state, 3: initial state, 4: initial output)"
        annotation(Dialog(tab="Advanced", group="PIDs general"));
      parameter Real tol = 0.1
        "tolerance [kW] for idle mode concerning heat transfer setpoint dotQ"
        annotation(Dialog(tab="Advanced", group="Miscellaneous"));

      // !!!!! variables !!!!!
      Real beta_prim_prod(min=0, max=1)
        "weight for the relevance of the error of the temperature(difference)";
      Real beta_sec_prod(min=0, max=1)
        "weight for the relevance of the error of the temperature(difference)";
      Real beta_prim_cons(min=0, max=1)
        "weight for the relevance of the error of the temperature(difference)";
      Real beta_sec_cons(min=0, max=1)
        "weight for the relevance of the error of the temperature(difference)";
      Integer  prosumer_mode
        "prosumer mode {-1;0;1}";
      Real T_prim_relev_des
          "desired value of relevant temperature (difference)
      for control of primary side";
      Real T_prim_relev_is
          "current value of relevant temperature (difference)
      for control of primary side";
      Real T_sec_relev_des
          "desired value of relevant temperature (difference)
      for control of secondary side";
      Real T_sec_relev_is
          "current value of relevant temperature (difference)
      for control of secondary side";

      Real PIDin_prim_cons_is_weighted
          "weighted input of is-values for PID_prim_cons";
      Real PIDin_prim_cons_des_weighted
          "weighted input of desired values for PID_prim_cons";
      Real PIDin_prim_prod_is_weighted
          "weighted input of is-values for PID_prim_prod";
      Real PIDin_prim_prod_des_weighted
          "weighted input of desired values for PID_prim_prod";
      Real PIDin_sec_cons_is_weighted
          "weighted input of is-values for PID_sec_cons";
      Real PIDin_sec_cons_des_weighted
          "weighted input of desired values for PID_sec_cons";
      Real PIDin_sec_prod_is_weighted
          "weighted input of is-values for PID_sec_prod";
      Real PIDin_sec_prod_des_weighted
          "weighted input of desired values for PID_sec_prod";

      Real error_prim_weighted
          "weighted overall error of primary side controller";

      Real error_sec_weighted
          "weighted overall error of primary side controller";

      Real error_T_prim_abs
          "temperature error of primary side controller";

      Real error_T_sec_abs
          "temperature error of primary side controller";

      Real error_Q_abs
          "temperature error of primary side controller";

      Real error_T_high_prio_abs
          "error of higher prioritized temperature objective";

      Real error_T_low_prio_abs
          "error of lower prioritized temperature objective";

      Real Delta_T_prim
          "weighted overall error of primary side controller";

      Real Delta_T_sec
          "weighted overall error of primary side controller";

      Real Q_dot_is_use;
      Real Q_dot_set_use;

       Real Delta_p_prim(unit="Pa", displayUnit="bar") annotation (
          Placement(transformation(
            extent={{-20,-20},{20,20}},
            rotation=180,
            origin={-80,-140})));

      // !!!!! ports !!!!!

      Modelica.Blocks.Interfaces.RealVectorInput states[8]
        annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));

      Modelica.Blocks.Interfaces.RealVectorOutput contr_vars_real[6]
        annotation (Placement(transformation(extent={{100,-20},{140,20}})));

      Real u_set
        "Normalized velocity of feed-in pump"
          annotation (Placement(transformation(
            extent={{-20,-20},{20,20}},
            rotation=0,
            origin={80,-60}),  iconTransformation(
            extent={{-20,-20},{20,20}},
            rotation=0,
            origin={100,-60})));
      Real kappa_set
        "Normalized flow coefficient for control valve"
         annotation (Placement(transformation(
            extent={{-20,-20},{20,20}},
            rotation=0,
            origin={80,-100}),iconTransformation(
            extent={{-20,-20},{20,20}},
            rotation=0,
            origin={100,-100})));
      Real pi_set
        "Participation" annotation (Placement(transformation(
            extent={{-20,-20},{20,20}},
            rotation=0,
            origin={80,20}),    iconTransformation(
            extent={{-20,-20},{20,20}},
            rotation=0,
            origin={100,20})));
      Real mu_set
        "Operating mode" annotation (Placement(transformation(
            extent={{-20,-20},{20,20}},
            rotation=0,
            origin={80,-20}),  iconTransformation(
            extent={{-20,-20},{20,20}},
            rotation=0,
            origin={100,-20})));
      Real T_sec_hot(
        unit="K",
        displayUnit="degC",
        min=277) "current temperature hot level secondary side"      annotation (Placement(
            transformation(
            extent={{-20,-20},{20,20}},
            rotation=0,
            origin={-80,60})));
      Real T_sec_cold(
        unit="K",
        displayUnit="degC",
        min=277)
               "current temperature cold  level secondary side"    annotation (Placement(
            transformation(
            extent={{-20,-20},{20,20}},
            rotation=0,
            origin={-80,20})));
      Real T_prim_hot(
        unit="K",
        displayUnit="degC",
        min=277)
               "current temperature hot level primary side"        annotation (Placement(
            transformation(
            extent={{-20,-20},{20,20}},
            rotation=0,
            origin={-80,140})));
      Real T_prim_cold(
        unit="K",
        displayUnit="degC",
        min=277)
                "current temperature cold level primary side"       annotation (Placement(
            transformation(
            extent={{-20,-20},{20,20}},
            rotation=0,
            origin={-80,100})));
      Modelica.Blocks.Interfaces.RealInput Q_dot_set(unit="kW", displayUnit="kW")
        "setpoint heat transfer (positive production, negative consumption)"
        annotation (Placement(transformation(extent={{-20,-20},{20,20}},
            rotation=-90,
            origin={-60,188})));
      Real V_dot_prim(unit="l/min", displayUnit="l/min")
        annotation (Placement(transformation(extent={{-100,-40},{-60,0}})));
      Real V_dot_sec(unit="l/min", displayUnit="l/min")
        annotation (Placement(transformation(extent={{-100,-80},{-60,-40}})));

      Real Q_dot_is(unit="kW", displayUnit="kW")
        "currently transferred heat (positive production, negative consumption)"
        annotation (Placement(transformation(extent={{-100,-120},{-60,-80}})));
      Modelica.Blocks.Continuous.LimPID PID_prim_cons(
        controllerType=controllerType,
        k=k_prim_cons,
        Ti=Ti_prim_cons,
        Td=Td_prim_cons,
        yMax=1,
        yMin=0.25,
        initType=initType,
        y_start=PID_prim_cons.yMax)
        annotation (Placement(transformation(extent={{-30,-32},{-10,-12}})));
      Modelica.Blocks.Continuous.LimPID PID_sec_cons(
        controllerType=controllerType,
        k=k_sec_cons,
        Ti=Ti_sec_cons,
        Td=Td_sec_cons,
        yMax=V_dot_sec_max,
        yMin=2,
        initType=initType,
        y_start=PID_sec_cons.yMax)
        annotation (Placement(transformation(extent={{-34,28},{-14,48}})));
      Modelica.Blocks.Continuous.LimPID PID_prim_prod(
        controllerType=controllerType,
        k=k_prim_prod,
        Ti=Ti_prim_prod,
        Td=Td_prim_prod,
        yMax=1,
        yMin=0.25,
        initType=initType,
        y_start=PID_prim_prod.yMax) annotation (Placement(transformation(extent={{12,-30},
                {32,-10}})));
      Modelica.Blocks.Continuous.LimPID PID_sec_prod(
        controllerType=controllerType,
        k=k_sec_prod,
        Ti=Ti_sec_prod,
        Td=Td_sec_prod,
        yMax=V_dot_sec_max,
        yMin=2,
        initType=initType,
        y_start=PID_sec_prod.yMax) annotation (Placement(transformation(extent={{10,28},
                {30,48}})));
      Real T_sec_set(unit="K", displayUnit="degC")
         "Temperature on the secondary side" annotation (Placement(transformation(
            extent={{-20,-20},{20,20}},
            rotation=0,
            origin={80,100})));
      Real V_dot_sec_set(unit="l/min", displayUnit=
           "l/min") "volume flow rate setpoint on the secondary side" annotation (Placement(transformation(
            extent={{-20,-20},{20,20}},
            rotation=0,
            origin={80,60})));
      Modelica.Blocks.Interfaces.RealInput T_sec_in_is(unit="K", displayUnit="degC")
        "Temperature on the secondary side" annotation (Placement(transformation(
            extent={{-20,-20},{20,20}},
            rotation=270,
            origin={60,190})));

    equation

      // assign inputs
      T_prim_hot   = states[1];
      T_prim_cold  = states[2];
      T_sec_hot    = states[3];
      T_sec_cold   = states[4];
      V_dot_prim   = states[5];
      V_dot_sec    = states[6];
      Q_dot_is     = states[7];
      Delta_p_prim = states[8];

      Delta_T_prim      = T_prim_hot -T_prim_cold;
      Delta_T_sec       = T_sec_hot  -T_sec_cold;

      beta_prim_prod = 1 - alpha_prim_prod;
      beta_sec_prod  = 1 - alpha_sec_prod;
      beta_prim_cons = 1 - alpha_prim_cons;
      beta_sec_cons  = 1 - alpha_sec_cons;

      // determine easy static values that just depend on prosumer mode
      // determine inputs for the four PIDs
      // four PIDs in order to be able to have different gains for each situation

      if  Q_dot_set <= 0-tol then // consumption mode
        prosumer_mode = -1;
      elseif Q_dot_set >= 0+tol then // production mode
        prosumer_mode = +1;
      else // idle mode
        prosumer_mode = 0;
      end if;

      Q_dot_is_use = abs(Q_dot_is);
      Q_dot_set_use = abs(Q_dot_set);

      if prosumer_mode == -1 then // consumption mode
        pi_set = 1;
        mu_set = -1;
        T_prim_relev_des = DeltaT_prim_des;
        T_prim_relev_is = T_prim_hot-T_prim_cold;
        T_sec_relev_des = T_sec_hot_des;
        T_sec_relev_is = T_sec_hot;

        PIDin_prim_cons_is_weighted    = alpha_prim_cons*Q_dot_is_use/Delta_Qdot_norm + beta_prim_cons*(-1)*T_prim_relev_is/Delta_T_norm;
        PIDin_prim_cons_des_weighted   = alpha_prim_cons*Q_dot_set_use/Delta_Qdot_norm + beta_prim_cons*(-1)*T_prim_relev_des/Delta_T_norm;
        PIDin_prim_prod_is_weighted    = 0;
        PIDin_prim_prod_des_weighted   = 0;
        PIDin_sec_cons_is_weighted     = alpha_sec_cons*Q_dot_is_use/Delta_Qdot_norm + beta_sec_cons*(-1)*T_sec_relev_is/Delta_T_norm;
        PIDin_sec_cons_des_weighted    = alpha_sec_cons*Q_dot_set_use/Delta_Qdot_norm + beta_sec_cons*(-1)*T_sec_relev_des/Delta_T_norm;
        PIDin_sec_prod_is_weighted     = 0;
        PIDin_sec_prod_des_weighted    = 0;

        error_prim_weighted            = PIDin_prim_cons_des_weighted - PIDin_prim_cons_is_weighted;
        error_sec_weighted             = PIDin_sec_cons_des_weighted - PIDin_sec_cons_is_weighted;

        error_T_prim_abs               = DeltaT_prim_des - Delta_T_prim;
        error_T_sec_abs                = T_sec_hot_des - T_sec_hot;

        error_T_high_prio_abs          = T_sec_hot_des - T_sec_hot;
        error_T_low_prio_abs           = DeltaT_prim_des - Delta_T_prim;

      elseif prosumer_mode == 1 then // production mode
        pi_set = 1;
        mu_set = 1;
        T_prim_relev_des = T_prim_hot_des;
        T_prim_relev_is = T_prim_hot;
        T_sec_relev_des = DeltaT_sec_des;
        T_sec_relev_is = T_sec_hot-T_sec_cold;

        PIDin_prim_cons_is_weighted    = 0;
        PIDin_prim_cons_des_weighted   = 0;
        PIDin_prim_prod_is_weighted    = alpha_prim_prod*Q_dot_is_use/Delta_Qdot_norm + beta_prim_prod*(-1)*T_prim_relev_is/Delta_T_norm;
        PIDin_prim_prod_des_weighted   = alpha_prim_prod*Q_dot_set_use/Delta_Qdot_norm + beta_prim_prod*(-1)*T_prim_relev_des/Delta_T_norm;
        PIDin_sec_cons_is_weighted     = 0;
        PIDin_sec_cons_des_weighted    = 0;
        PIDin_sec_prod_is_weighted     = alpha_sec_prod*Q_dot_is_use/Delta_Qdot_norm + beta_sec_prod*(-1)*T_sec_relev_is/Delta_T_norm;
        PIDin_sec_prod_des_weighted    = alpha_sec_prod*Q_dot_set_use/Delta_Qdot_norm + beta_sec_prod*(-1)*T_sec_relev_des/Delta_T_norm;

        error_prim_weighted            = PIDin_prim_prod_des_weighted - PIDin_prim_prod_is_weighted;
        error_sec_weighted             = PIDin_sec_prod_des_weighted - PIDin_sec_prod_is_weighted;

        error_T_prim_abs               = T_prim_hot_des - T_prim_hot;
        error_T_sec_abs                = DeltaT_sec_des - Delta_T_sec;

        error_T_high_prio_abs          = T_prim_hot_des - T_prim_hot;
        error_T_low_prio_abs           = DeltaT_sec_des - Delta_T_sec;

      else // idle mode
        pi_set = 0;
        mu_set = -1;
        T_prim_relev_des = 0;
        T_prim_relev_is = 0;
        T_sec_relev_des = 0;
        T_sec_relev_is = 0;

        PIDin_prim_cons_is_weighted    = 0;
        PIDin_prim_cons_des_weighted   = 0;
        PIDin_prim_prod_is_weighted    = 0;
        PIDin_prim_prod_des_weighted   = 0;
        PIDin_sec_cons_is_weighted     = 0;
        PIDin_sec_cons_des_weighted    = 0;
        PIDin_sec_prod_is_weighted     = 0;
        PIDin_sec_prod_des_weighted    = 0;

        error_prim_weighted            = 0;
        error_sec_weighted             = 0;

        error_T_prim_abs               = 0;
        error_T_sec_abs                = 0;

        error_T_high_prio_abs          = 0;
        error_T_low_prio_abs           = 0;

      end if;

      // assign PID controller inputs
      PID_prim_cons.u_s    = PIDin_prim_cons_des_weighted;
      PID_prim_cons.u_m    = PIDin_prim_cons_is_weighted;
      PID_prim_prod.u_s    = PIDin_prim_prod_des_weighted;
      PID_prim_prod.u_m    = PIDin_prim_prod_is_weighted;
      PID_sec_cons.u_s     = PIDin_sec_cons_des_weighted;
      PID_sec_cons.u_m     = PIDin_sec_cons_is_weighted;
      PID_sec_prod.u_s     = PIDin_sec_prod_des_weighted;
      PID_sec_prod.u_m     = PIDin_sec_prod_is_weighted;

      // connect secondary side temperature setpoint
      T_sec_set    =  T_sec_in_is;

      // assign PID outputs to controller outputs
      if prosumer_mode == -1 then // consumption mode
        u_set = 0;
        kappa_set =PID_prim_cons.y;
        V_dot_sec_set = PID_sec_cons.y;
      elseif prosumer_mode == 1 then // production mode
        u_set =PID_prim_prod.y;
        kappa_set = 0;
        V_dot_sec_set = PID_sec_prod.y;
      else // idle mode
        V_dot_sec_set = 0;
        u_set = 0;
        kappa_set = 0;
      end if;

      error_Q_abs = Q_dot_set_use - Q_dot_is_use;

      // assign control variables vector
      contr_vars_real[1]   =  T_sec_set;
      contr_vars_real[2]   =  V_dot_sec_set;
      contr_vars_real[3]   =  pi_set;
      contr_vars_real[4]   =  mu_set;
      contr_vars_real[5]   =  u_set;
      contr_vars_real[6]   =  kappa_set;

        annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-120,-160},{
                120,180}}),                                           graphics={
              Text(
              extent={{-70,56},{64,-56}},
              textColor={28,108,200},
              textString="weighted
PID"),     Rectangle(extent={{-120,180},{120,-160}}, lineColor={0,0,0})}),
                                                                       Diagram(
            coordinateSystem(preserveAspectRatio=false, extent={{-120,-160},{120,180}})));
    end PID_Q_T_weighted_sameside;

    model PID_Q_T_weighted_crossover

      import Modelica.Units.SI;
      import T_AbsZeroDegC = Modelica.Constants.T_zero;
      import Modelica.Blocks.Types.Init;
      import Modelica.Blocks.Types.SimpleController;

        // !!!!! parameters !!!!!
      parameter Real Delta_Qdot_norm = 1
          "Heat power value for normalizing the error (deviation) of the transferred heat.
      For alpha=0.5 a deviation of Delta_Qdot_norm in heat transfer is weigthed equal to a deviation of Delta_T_norm in temperature."
          annotation(Dialog(group="Normalizing values"));
      parameter SI.TemperatureDifference Delta_T_norm(min=0) = 3
          "Temperature difference for normalizing the error (deviation) of the temperature.
      For alpha=0.5 a deviation of Delta_T_norm in temperature is weighted equal to a deviation of Delta_Qdot_norm in heat transfer."
          annotation(Dialog(group="Normalizing values"));

      parameter SI.Temperature T_prim_hot_des(min=277)= - T_AbsZeroDegC + 53.5
          "desired temperature supply primary side"
          annotation(Dialog(group="Temperature objectives"));
      parameter SI.Temperature T_sec_hot_des(min=277)= - T_AbsZeroDegC + 50
          "desired temperature supply secondary side"
          annotation(Dialog(group="Temperature objectives"));
      parameter SI.TemperatureDifference DeltaT_prim_des(min=1) =   20
          "desired temperature difference primary side"
          annotation(Dialog(group="Temperature objectives"));
      parameter SI.TemperatureDifference DeltaT_sec_des(min=1) =   20
          "desired temperature difference secondary side"
          annotation(Dialog(group="Temperature objectives"));
      parameter Real V_dot_sec_max(unit="l/min", displayUnit="l/min") = 8.5
        "maximum secondary side volume flow in [l/min]"
        annotation(Dialog(group="General PID settings"));
      parameter Real k_prim_prod = 1.5
        "Proportional gain for controller in [-]"
        annotation(Dialog(group="PID primary side - producer mode - tuning"));
      parameter Real Ti_prim_prod = 8
        "Integral time constant for controller in [s]"
        annotation(Dialog(group="PID primary side - producer mode - tuning"));
      parameter Real Td_prim_prod = 0
        "Derivative time constant for controller in [s]"
        annotation(Dialog(group="PID primary side - producer mode - tuning"));
      parameter Real alpha_prim_prod(min=0, max=1) = 0.666
        "weight for the relevance of the error of the transferred heat in comparison to the error of temperature objectives (sum is one)"
        annotation(Dialog(group="PID primary side - producer mode - tuning"));
      parameter Real k_sec_prod = 1.5
        "Proportional gain for controller in [-]"
        annotation(Dialog(group="PID secondary side - producer mode - tuning"));
      parameter Real Ti_sec_prod = 8
        "Integral time constant for controller in [s]"
        annotation(Dialog(group="PID secondary side - producer mode - tuning"));
      parameter Real Td_sec_prod = 0
        "Derivative time constant for controller in [s]"
        annotation(Dialog(group="PID secondary side - producer mode - tuning"));
      parameter Real alpha_sec_prod(min=0, max=1) = 0.333
        "weight for the relevance of the error of the transferred heat in comparison to the error of temperature objectives (sum is one)"
        annotation(Dialog(group="PID secondary side - producer mode - tuning"));
      parameter Real k_prim_cons = 1
        "Proportional gain for controller in [-]"
        annotation(Dialog(group="PID primary side - consumer mode - tuning"));
      parameter Real Ti_prim_cons = 35
        "Integral time constant for controller in [s]"
        annotation(Dialog(group="PID primary side - consumer mode - tuning"));
      parameter Real Td_prim_cons = 0
        "Derivative time constant for controller in [s]"
        annotation(Dialog(group="PID primary side - consumer mode - tuning"));
      parameter Real alpha_prim_cons(min=0, max=1) = 0.333
        "weight for the relevance of the error of the transferred heat in comparison to the error of temperature objectives (sum is one)"
        annotation(Dialog(group="PID primary side - consumer mode - tuning"));
      parameter Real k_sec_cons = 1.5
        "Proportional gain for controller in [-]"
        annotation(Dialog(group="PID secondary side - consumer mode - tuning"));
      parameter Real Ti_sec_cons = 8
        "Integral time constant for controller in [s]"
        annotation(Dialog(group="PID secondary side - consumer mode - tuning"));
      parameter Real Td_sec_cons = 0
        "Derivative time constant for controller in [s]"
        annotation(Dialog(group="PID secondary side - consumer mode - tuning"));
      parameter Real alpha_sec_cons(min=0, max=1) = 0.666
        "weight for the relevance of the error of the transferred heat in comparison to the error of temperature objectives (sum is one)"
        annotation(Dialog(group="PID secondary side - consumer mode - tuning"));
      parameter .Modelica.Blocks.Types.SimpleController controllerType=
             Modelica.Blocks.Types.SimpleController.PID "Type of controller"
             annotation(Dialog(tab="Advanced", group="PIDs general"));
      parameter Init initType = Modelica.Blocks.Types.Init.NoInit
        "Type of initialization (1: no init, 2: steady state, 3: initial state, 4: initial output)"
        annotation(Dialog(tab="Advanced", group="PIDs general"));
      parameter Real tol = 0.1
        "tolerance [kW] for idle mode concerning heat transfer setpoint dotQ"
        annotation(Dialog(tab="Advanced", group="Miscellaneous"));

      // !!!!! variables !!!!!
      Real beta_prim_prod(min=0, max=1)
        "weight for the relevance of the error of the temperature(difference)";
      Real beta_sec_prod(min=0, max=1)
        "weight for the relevance of the error of the temperature(difference)";
      Real beta_prim_cons(min=0, max=1)
        "weight for the relevance of the error of the temperature(difference)";
      Real beta_sec_cons(min=0, max=1)
        "weight for the relevance of the error of the temperature(difference)";
      Integer  prosumer_mode
        "prosumer mode {-1;0;1}";
      Real T_prim_relev_des
          "desired value of relevant temperature (difference)
      for control of primary side";
      Real T_prim_relev_is
          "current value of relevant temperature (difference)
      for control of primary side";
      Real T_sec_relev_des
          "desired value of relevant temperature (difference)
      for control of secondary side";
      Real T_sec_relev_is
          "current value of relevant temperature (difference)
      for control of secondary side";

      Real PIDin_prim_cons_is_weighted
          "weighted input of is-values for PID_prim_cons";
      Real PIDin_prim_cons_des_weighted
          "weighted input of desired values for PID_prim_cons";
      Real PIDin_prim_prod_is_weighted
          "weighted input of is-values for PID_prim_prod";
      Real PIDin_prim_prod_des_weighted
          "weighted input of desired values for PID_prim_prod";
      Real PIDin_sec_cons_is_weighted
          "weighted input of is-values for PID_sec_cons";
      Real PIDin_sec_cons_des_weighted
          "weighted input of desired values for PID_sec_cons";
      Real PIDin_sec_prod_is_weighted
          "weighted input of is-values for PID_sec_prod";
      Real PIDin_sec_prod_des_weighted
          "weighted input of desired values for PID_sec_prod";

      Real error_prim_weighted
          "weighted overall error of primary side controller";

      Real error_sec_weighted
          "weighted overall error of primary side controller";

      Real error_T_prim_abs
          "temperature error of primary side controller";

      Real error_T_sec_abs
          "temperature error of primary side controller";

      Real error_Q_abs
          "temperature error of primary side controller";

      Real error_T_high_prio_abs
          "error of higher prioritized temperature objective";

      Real error_T_low_prio_abs
          "error of lower prioritized temperature objective";

      Real Delta_T_prim
          "weighted overall error of primary side controller";

      Real Delta_T_sec
          "weighted overall error of primary side controller";

      Real Q_dot_is_use;
      Real Q_dot_set_use;

       Real Delta_p_prim(unit="Pa", displayUnit="bar") annotation (
          Placement(transformation(
            extent={{-20,-20},{20,20}},
            rotation=180,
            origin={-80,-140})));

      // !!!!! ports !!!!!

      Modelica.Blocks.Interfaces.RealVectorInput states[8]
        annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));

      Modelica.Blocks.Interfaces.RealVectorOutput contr_vars_real[6]
        annotation (Placement(transformation(extent={{100,-20},{140,20}})));

      Real u_set
        "Normalized velocity of feed-in pump"
          annotation (Placement(transformation(
            extent={{-20,-20},{20,20}},
            rotation=0,
            origin={80,-60}),  iconTransformation(
            extent={{-20,-20},{20,20}},
            rotation=0,
            origin={100,-60})));
      Real kappa_set
        "Normalized flow coefficient for control valve"
         annotation (Placement(transformation(
            extent={{-20,-20},{20,20}},
            rotation=0,
            origin={80,-100}),iconTransformation(
            extent={{-20,-20},{20,20}},
            rotation=0,
            origin={100,-100})));
      Real pi_set
        "Participation" annotation (Placement(transformation(
            extent={{-20,-20},{20,20}},
            rotation=0,
            origin={80,20}),    iconTransformation(
            extent={{-20,-20},{20,20}},
            rotation=0,
            origin={100,20})));
      Real mu_set
        "Operating mode" annotation (Placement(transformation(
            extent={{-20,-20},{20,20}},
            rotation=0,
            origin={80,-20}),  iconTransformation(
            extent={{-20,-20},{20,20}},
            rotation=0,
            origin={100,-20})));
      Real T_sec_hot(
        unit="K",
        displayUnit="degC",
        min=277) "current temperature hot level secondary side"      annotation (Placement(
            transformation(
            extent={{-20,-20},{20,20}},
            rotation=0,
            origin={-80,60})));
      Real T_sec_cold(
        unit="K",
        displayUnit="degC",
        min=277)
               "current temperature cold  level secondary side"    annotation (Placement(
            transformation(
            extent={{-20,-20},{20,20}},
            rotation=0,
            origin={-80,20})));
      Real T_prim_hot(
        unit="K",
        displayUnit="degC",
        min=277)
               "current temperature hot level primary side"        annotation (Placement(
            transformation(
            extent={{-20,-20},{20,20}},
            rotation=0,
            origin={-80,140})));
      Real T_prim_cold(
        unit="K",
        displayUnit="degC",
        min=277)
                "current temperature cold level primary side"       annotation (Placement(
            transformation(
            extent={{-20,-20},{20,20}},
            rotation=0,
            origin={-80,100})));
      Modelica.Blocks.Interfaces.RealInput Q_dot_set(unit="kW", displayUnit="kW")
        "setpoint heat transfer (positive production, negative consumption)"
        annotation (Placement(transformation(extent={{-20,-20},{20,20}},
            rotation=-90,
            origin={-60,188})));
      Real V_dot_prim(unit="l/min", displayUnit="l/min")
        annotation (Placement(transformation(extent={{-100,-40},{-60,0}})));
      Real V_dot_sec(unit="l/min", displayUnit="l/min")
        annotation (Placement(transformation(extent={{-100,-80},{-60,-40}})));

      Real Q_dot_is(unit="kW", displayUnit="kW")
        "currently transferred heat (positive production, negative consumption)"
        annotation (Placement(transformation(extent={{-100,-120},{-60,-80}})));
      Modelica.Blocks.Continuous.LimPID PID_prim_cons(
        controllerType=controllerType,
        k=k_prim_cons,
        Ti=Ti_prim_cons,
        Td=Td_prim_cons,
        yMax=1,
        yMin=0.25,
        initType=initType,
        y_start=PID_prim_cons.yMax)
        annotation (Placement(transformation(extent={{-30,-32},{-10,-12}})));
      Modelica.Blocks.Continuous.LimPID PID_sec_cons(
        controllerType=controllerType,
        k=k_sec_cons,
        Ti=Ti_sec_cons,
        Td=Td_sec_cons,
        yMax=V_dot_sec_max,
        yMin=2,
        initType=initType,
        y_start=PID_sec_cons.yMax)
        annotation (Placement(transformation(extent={{-34,28},{-14,48}})));
      Modelica.Blocks.Continuous.LimPID PID_prim_prod(
        controllerType=controllerType,
        k=k_prim_prod,
        Ti=Ti_prim_prod,
        Td=Td_prim_prod,
        yMax=1,
        yMin=0.25,
        initType=initType,
        y_start=PID_prim_prod.yMax) annotation (Placement(transformation(extent={{12,-30},
                {32,-10}})));
      Modelica.Blocks.Continuous.LimPID PID_sec_prod(
        controllerType=controllerType,
        k=k_sec_prod,
        Ti=Ti_sec_prod,
        Td=Td_sec_prod,
        yMax=V_dot_sec_max,
        yMin=2,
        initType=initType,
        y_start=PID_sec_prod.yMax) annotation (Placement(transformation(extent={{10,28},
                {30,48}})));
      Real T_sec_set(unit="K", displayUnit="degC")
         "Temperature on the secondary side" annotation (Placement(transformation(
            extent={{-20,-20},{20,20}},
            rotation=0,
            origin={80,100})));
      Real V_dot_sec_set(unit="l/min", displayUnit=
           "l/min") "volume flow rate setpoint on the secondary side" annotation (Placement(transformation(
            extent={{-20,-20},{20,20}},
            rotation=0,
            origin={80,60})));
      Modelica.Blocks.Interfaces.RealInput T_sec_in_is(unit="K", displayUnit="degC")
        "Temperature on the secondary side" annotation (Placement(transformation(
            extent={{-20,-20},{20,20}},
            rotation=270,
            origin={60,190})));

    equation

      // assign inputs
      T_prim_hot   = states[1];
      T_prim_cold  = states[2];
      T_sec_hot    = states[3];
      T_sec_cold   = states[4];
      V_dot_prim   = states[5];
      V_dot_sec    = states[6];
      Q_dot_is     = states[7];
      Delta_p_prim = states[8];

      Delta_T_prim      = T_prim_hot -T_prim_cold;
      Delta_T_sec       = T_sec_hot  -T_sec_cold;

      beta_prim_prod = 1 - alpha_prim_prod;
      beta_sec_prod  = 1 - alpha_sec_prod;
      beta_prim_cons = 1 - alpha_prim_cons;
      beta_sec_cons  = 1 - alpha_sec_cons;

      // determine easy static values that just depend on prosumer mode
      // determine inputs for the four PIDs
      // four PIDs in order to be able to have different gains for each situation

      if  Q_dot_set <= 0-tol then // consumption mode
        prosumer_mode = -1;
      elseif Q_dot_set >= 0+tol then // production mode
        prosumer_mode = +1;
      else // idle mode
        prosumer_mode = 0;
      end if;

      Q_dot_is_use = abs(Q_dot_is);
      Q_dot_set_use = abs(Q_dot_set);

      if prosumer_mode == -1 then // consumption mode
        pi_set = 1;
        mu_set = -1;
        T_prim_relev_des = T_sec_hot_des;
        T_prim_relev_is = T_sec_hot;
        T_sec_relev_des = DeltaT_prim_des;
        T_sec_relev_is = T_prim_hot-T_prim_cold;

        PIDin_prim_cons_is_weighted    = alpha_prim_cons*Q_dot_is_use/Delta_Qdot_norm + beta_prim_cons*T_prim_relev_is/Delta_T_norm;
        PIDin_prim_cons_des_weighted   = alpha_prim_cons*Q_dot_set_use/Delta_Qdot_norm + beta_prim_cons*T_prim_relev_des/Delta_T_norm;
        PIDin_prim_prod_is_weighted    = 0;
        PIDin_prim_prod_des_weighted   = 0;
        PIDin_sec_cons_is_weighted     = alpha_sec_cons*Q_dot_is_use/Delta_Qdot_norm + beta_sec_cons*T_sec_relev_is/Delta_T_norm;
        PIDin_sec_cons_des_weighted    = alpha_sec_cons*Q_dot_set_use/Delta_Qdot_norm +
          beta_sec_cons*T_sec_relev_des/Delta_T_norm;
        PIDin_sec_prod_is_weighted     = 0;
        PIDin_sec_prod_des_weighted    = 0;

        error_prim_weighted            = PIDin_prim_cons_des_weighted - PIDin_prim_cons_is_weighted;
        error_sec_weighted             = PIDin_sec_cons_des_weighted - PIDin_sec_cons_is_weighted;

        error_T_prim_abs               = DeltaT_prim_des - Delta_T_prim;
        error_T_sec_abs                = T_sec_hot_des - T_sec_hot;

        error_T_high_prio_abs          = T_sec_hot_des - T_sec_hot; // T_sec_hot
        error_T_low_prio_abs           = DeltaT_prim_des - Delta_T_prim; // Delta_T_prim

      elseif prosumer_mode == 1 then // production mode
        pi_set = 1;
        mu_set = 1;
        T_prim_relev_des = DeltaT_sec_des;
        T_prim_relev_is = T_sec_hot-T_sec_cold;
        T_sec_relev_des = T_prim_hot_des;
        T_sec_relev_is = T_prim_hot;

        PIDin_prim_cons_is_weighted    = 0;
        PIDin_prim_cons_des_weighted   = 0;
        PIDin_prim_prod_is_weighted    = alpha_prim_prod*Q_dot_is_use/Delta_Qdot_norm + beta_prim_prod*T_prim_relev_is/Delta_T_norm;
        PIDin_prim_prod_des_weighted   = alpha_prim_prod*Q_dot_set_use/Delta_Qdot_norm + beta_prim_prod*T_prim_relev_des/Delta_T_norm;
        PIDin_sec_cons_is_weighted     = 0;
        PIDin_sec_cons_des_weighted    = 0;
        PIDin_sec_prod_is_weighted     = alpha_sec_prod*Q_dot_is_use/Delta_Qdot_norm + beta_sec_prod*T_sec_relev_is/Delta_T_norm;
        PIDin_sec_prod_des_weighted    = alpha_sec_prod*Q_dot_set_use/Delta_Qdot_norm + beta_sec_prod*T_sec_relev_des/Delta_T_norm;

        error_prim_weighted            = PIDin_prim_prod_des_weighted - PIDin_prim_prod_is_weighted;
        error_sec_weighted             = PIDin_sec_prod_des_weighted - PIDin_sec_prod_is_weighted;

        error_T_prim_abs               = T_prim_hot_des - T_prim_hot;
        error_T_sec_abs                = DeltaT_sec_des - Delta_T_sec;

        error_T_high_prio_abs          = T_prim_hot_des - T_prim_hot; // T_pim_hot
        error_T_low_prio_abs           = DeltaT_sec_des - Delta_T_sec; // Deltat_T_sec

      else // idle mode
        pi_set = 0;
        mu_set = -1;
        T_prim_relev_des = 0;
        T_prim_relev_is = 0;
        T_sec_relev_des = 0;
        T_sec_relev_is = 0;

        PIDin_prim_cons_is_weighted    = 0;
        PIDin_prim_cons_des_weighted   = 0;
        PIDin_prim_prod_is_weighted    = 0;
        PIDin_prim_prod_des_weighted   = 0;
        PIDin_sec_cons_is_weighted     = 0;
        PIDin_sec_cons_des_weighted    = 0;
        PIDin_sec_prod_is_weighted     = 0;
        PIDin_sec_prod_des_weighted    = 0;

        error_prim_weighted            = 0;
        error_sec_weighted             = 0;

        error_T_prim_abs               = 0;
        error_T_sec_abs                = 0;

        error_T_high_prio_abs          = 0;
        error_T_low_prio_abs           = 0;

      end if;

      // assign PID controller inputs
      PID_prim_cons.u_s    = PIDin_prim_cons_des_weighted;
      PID_prim_cons.u_m    = PIDin_prim_cons_is_weighted;
      PID_prim_prod.u_s    = PIDin_prim_prod_des_weighted;
      PID_prim_prod.u_m    = PIDin_prim_prod_is_weighted;
      PID_sec_cons.u_s     = PIDin_sec_cons_des_weighted;
      PID_sec_cons.u_m     = PIDin_sec_cons_is_weighted;
      PID_sec_prod.u_s     = PIDin_sec_prod_des_weighted;
      PID_sec_prod.u_m     = PIDin_sec_prod_is_weighted;

      // connect secondary side temperature setpoint
      T_sec_set    =  T_sec_in_is;

      // assign PID outputs to controller outputs
      if prosumer_mode == -1 then // consumption mode
        u_set = 0;
        kappa_set =PID_prim_cons.y;
        V_dot_sec_set = PID_sec_cons.y;
      elseif prosumer_mode == 1 then // production mode
        u_set =PID_prim_prod.y;
        kappa_set = 0;
        V_dot_sec_set = PID_sec_prod.y;
      else // idle mode
        V_dot_sec_set = 0;
        u_set = 0;
        kappa_set = 0;
      end if;

      error_Q_abs = Q_dot_set_use - Q_dot_is_use;

      // assign control variables vector
      contr_vars_real[1]   =  T_sec_set;
      contr_vars_real[2]   =  V_dot_sec_set;
      contr_vars_real[3]   =  pi_set;
      contr_vars_real[4]   =  mu_set;
      contr_vars_real[5]   =  u_set;
      contr_vars_real[6]   =  kappa_set;

        annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-120,-160},{
                120,180}}),                                           graphics={
              Text(
              extent={{-70,56},{64,-56}},
              textColor={28,108,200},
              textString="weighted
PID"),     Rectangle(extent={{-120,180},{120,-160}}, lineColor={0,0,0})}),
                                                                       Diagram(
            coordinateSystem(preserveAspectRatio=false, extent={{-120,-160},{120,180}})));
    end PID_Q_T_weighted_crossover;

    model PID_Q_T_weighted_mix1

      import Modelica.Units.SI;
      import T_AbsZeroDegC = Modelica.Constants.T_zero;
      import Modelica.Blocks.Types.Init;
      import Modelica.Blocks.Types.SimpleController;

        // !!!!! parameters !!!!!
      parameter Real Delta_Qdot_norm = 1
          "Heat power value for normalizing the error (deviation) of the transferred heat.
      For alpha=0.5 a deviation of Delta_Qdot_norm in heat transfer is weigthed equal to a deviation of Delta_T_norm in temperature."
          annotation(Dialog(group="Normalizing values"));
      parameter SI.TemperatureDifference Delta_T_norm(min=0) = 3
          "Temperature difference for normalizing the error (deviation) of the temperature.
      For alpha=0.5 a deviation of Delta_T_norm in temperature is weighted equal to a deviation of Delta_Qdot_norm in heat transfer."
          annotation(Dialog(group="Normalizing values"));

      parameter SI.Temperature T_prim_hot_des(min=277)= - T_AbsZeroDegC + 53.5
          "desired temperature supply primary side"
          annotation(Dialog(group="Temperature objectives"));
      parameter SI.Temperature T_sec_hot_des(min=277)= - T_AbsZeroDegC + 50
          "desired temperature supply secondary side"
          annotation(Dialog(group="Temperature objectives"));
      parameter SI.TemperatureDifference DeltaT_prim_des(min=1) =   20
          "desired temperature difference primary side"
          annotation(Dialog(group="Temperature objectives"));
      parameter SI.TemperatureDifference DeltaT_sec_des(min=1) =   20
          "desired temperature difference secondary side"
          annotation(Dialog(group="Temperature objectives"));
      parameter Real V_dot_sec_max(unit="l/min", displayUnit="l/min") = 8.5
        "maximum secondary side volume flow in [l/min]"
        annotation(Dialog(group="General PID settings"));
      parameter Real k_prim_prod = 1.5
        "Proportional gain for controller in [-]"
        annotation(Dialog(group="PID primary side - producer mode - tuning"));
      parameter Real Ti_prim_prod = 8
        "Integral time constant for controller in [s]"
        annotation(Dialog(group="PID primary side - producer mode - tuning"));
      parameter Real Td_prim_prod = 0
        "Derivative time constant for controller in [s]"
        annotation(Dialog(group="PID primary side - producer mode - tuning"));
      parameter Real alpha_prim_prod(min=0, max=1) = 0.333
        "weight for the relevance of the error of the transferred heat in comparison to the error of temperature objectives (sum is one)"
        annotation(Dialog(group="PID primary side - producer mode - tuning"));
      parameter Real k_sec_prod = 1.5
        "Proportional gain for controller in [-]"
        annotation(Dialog(group="PID secondary side - producer mode - tuning"));
      parameter Real Ti_sec_prod = 8
        "Integral time constant for controller in [s]"
        annotation(Dialog(group="PID secondary side - producer mode - tuning"));
      parameter Real Td_sec_prod = 0
        "Derivative time constant for controller in [s]"
        annotation(Dialog(group="PID secondary side - producer mode - tuning"));
      parameter Real alpha_sec_prod(min=0, max=1) = 0.666
        "weight for the relevance of the error of the transferred heat in comparison to the error of temperature objectives (sum is one)"
        annotation(Dialog(group="PID secondary side - producer mode - tuning"));
      parameter Real k_prim_cons = 1.0
        "Proportional gain for controller in [-]"
        annotation(Dialog(group="PID primary side - consumer mode - tuning"));
      parameter Real Ti_prim_cons = 35
        "Integral time constant for controller in [s]"
        annotation(Dialog(group="PID primary side - consumer mode - tuning"));
      parameter Real Td_prim_cons = 0
        "Derivative time constant for controller in [s]"
        annotation(Dialog(group="PID primary side - consumer mode - tuning"));
      parameter Real alpha_prim_cons(min=0, max=1) = 0.333
        "weight for the relevance of the error of the transferred heat in comparison to the error of temperature objectives (sum is one)"
        annotation(Dialog(group="PID primary side - consumer mode - tuning"));
      parameter Real k_sec_cons = 1.5
        "Proportional gain for controller in [-]"
        annotation(Dialog(group="PID secondary side - consumer mode - tuning"));
      parameter Real Ti_sec_cons = 8
        "Integral time constant for controller in [s]"
        annotation(Dialog(group="PID secondary side - consumer mode - tuning"));
      parameter Real Td_sec_cons = 0
        "Derivative time constant for controller in [s]"
        annotation(Dialog(group="PID secondary side - consumer mode - tuning"));
      parameter Real alpha_sec_cons(min=0, max=1) = 0.666
        "weight for the relevance of the error of the transferred heat in comparison to the error of temperature objectives (sum is one)"
        annotation(Dialog(group="PID secondary side - consumer mode - tuning"));
      parameter .Modelica.Blocks.Types.SimpleController controllerType=
             Modelica.Blocks.Types.SimpleController.PID "Type of controller"
             annotation(Dialog(tab="Advanced", group="PIDs general"));
      parameter Init initType = Modelica.Blocks.Types.Init.NoInit
        "Type of initialization (1: no init, 2: steady state, 3: initial state, 4: initial output)"
        annotation(Dialog(tab="Advanced", group="PIDs general"));
      parameter Real tol = 0.1
        "tolerance [kW] for idle mode concerning heat transfer setpoint dotQ"
        annotation(Dialog(tab="Advanced", group="Miscellaneous"));

      // !!!!! variables !!!!!
      Real beta_prim_prod(min=0, max=1)
        "weight for the relevance of the error of the temperature(difference)";
      Real beta_sec_prod(min=0, max=1)
        "weight for the relevance of the error of the temperature(difference)";
      Real beta_prim_cons(min=0, max=1)
        "weight for the relevance of the error of the temperature(difference)";
      Real beta_sec_cons(min=0, max=1)
        "weight for the relevance of the error of the temperature(difference)";
      Integer  prosumer_mode
        "prosumer mode {-1;0;1}";
      Real T_prim_relev_des
          "desired value of relevant temperature (difference)
      for control of primary side";
      Real T_prim_relev_is
          "current value of relevant temperature (difference)
      for control of primary side";
      Real T_sec_relev_des
          "desired value of relevant temperature (difference)
      for control of secondary side";
      Real T_sec_relev_is
          "current value of relevant temperature (difference)
      for control of secondary side";

      Real PIDin_prim_cons_is_weighted
          "weighted input of is-values for PID_prim_cons";
      Real PIDin_prim_cons_des_weighted
          "weighted input of desired values for PID_prim_cons";
      Real PIDin_prim_prod_is_weighted
          "weighted input of is-values for PID_prim_prod";
      Real PIDin_prim_prod_des_weighted
          "weighted input of desired values for PID_prim_prod";
      Real PIDin_sec_cons_is_weighted
          "weighted input of is-values for PID_sec_cons";
      Real PIDin_sec_cons_des_weighted
          "weighted input of desired values for PID_sec_cons";
      Real PIDin_sec_prod_is_weighted
          "weighted input of is-values for PID_sec_prod";
      Real PIDin_sec_prod_des_weighted
          "weighted input of desired values for PID_sec_prod";

      Real error_prim_weighted
          "weighted overall error of primary side controller";

      Real error_sec_weighted
          "weighted overall error of primary side controller";

      Real error_T_prim_abs
          "temperature error of primary side controller";

      Real error_T_sec_abs
          "temperature error of primary side controller";

      Real error_Q_abs
          "temperature error of primary side controller";

      Real error_T_high_prio_abs
          "error of higher prioritized temperature objective";

      Real error_T_low_prio_abs
          "error of lower prioritized temperature objective";

      Real Delta_T_prim
          "weighted overall error of primary side controller";

      Real Delta_T_sec
          "weighted overall error of primary side controller";

      Real Q_dot_is_use;
      Real Q_dot_set_use;

       Real Delta_p_prim(unit="Pa", displayUnit="bar") annotation (
          Placement(transformation(
            extent={{-20,-20},{20,20}},
            rotation=180,
            origin={-80,-140})));

      // !!!!! ports !!!!!

      Modelica.Blocks.Interfaces.RealVectorInput states[8]
        annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));

      Modelica.Blocks.Interfaces.RealVectorOutput contr_vars_real[6]
        annotation (Placement(transformation(extent={{100,-20},{140,20}})));

      Real u_set
        "Normalized velocity of feed-in pump"
          annotation (Placement(transformation(
            extent={{-20,-20},{20,20}},
            rotation=0,
            origin={80,-60}),  iconTransformation(
            extent={{-20,-20},{20,20}},
            rotation=0,
            origin={100,-60})));
      Real kappa_set
        "Normalized flow coefficient for control valve"
         annotation (Placement(transformation(
            extent={{-20,-20},{20,20}},
            rotation=0,
            origin={80,-100}),iconTransformation(
            extent={{-20,-20},{20,20}},
            rotation=0,
            origin={100,-100})));
      Real pi_set
        "Participation" annotation (Placement(transformation(
            extent={{-20,-20},{20,20}},
            rotation=0,
            origin={80,20}),    iconTransformation(
            extent={{-20,-20},{20,20}},
            rotation=0,
            origin={100,20})));
      Real mu_set
        "Operating mode" annotation (Placement(transformation(
            extent={{-20,-20},{20,20}},
            rotation=0,
            origin={80,-20}),  iconTransformation(
            extent={{-20,-20},{20,20}},
            rotation=0,
            origin={100,-20})));
      Real T_sec_hot(
        unit="K",
        displayUnit="degC",
        min=277) "current temperature hot level secondary side"      annotation (Placement(
            transformation(
            extent={{-20,-20},{20,20}},
            rotation=0,
            origin={-80,60})));
      Real T_sec_cold(
        unit="K",
        displayUnit="degC",
        min=277)
               "current temperature cold  level secondary side"    annotation (Placement(
            transformation(
            extent={{-20,-20},{20,20}},
            rotation=0,
            origin={-80,20})));
      Real T_prim_hot(
        unit="K",
        displayUnit="degC",
        min=277)
               "current temperature hot level primary side"        annotation (Placement(
            transformation(
            extent={{-20,-20},{20,20}},
            rotation=0,
            origin={-80,140})));
      Real T_prim_cold(
        unit="K",
        displayUnit="degC",
        min=277)
                "current temperature cold level primary side"       annotation (Placement(
            transformation(
            extent={{-20,-20},{20,20}},
            rotation=0,
            origin={-80,100})));
      Modelica.Blocks.Interfaces.RealInput Q_dot_set(unit="kW", displayUnit="kW")
        "setpoint heat transfer (positive production, negative consumption)"
        annotation (Placement(transformation(extent={{-20,-20},{20,20}},
            rotation=-90,
            origin={-60,188})));
      Real V_dot_prim(unit="l/min", displayUnit="l/min")
        annotation (Placement(transformation(extent={{-100,-40},{-60,0}})));
      Real V_dot_sec(unit="l/min", displayUnit="l/min")
        annotation (Placement(transformation(extent={{-100,-80},{-60,-40}})));

      Real Q_dot_is(unit="kW", displayUnit="kW")
        "currently transferred heat (positive production, negative consumption)"
        annotation (Placement(transformation(extent={{-100,-120},{-60,-80}})));
      Modelica.Blocks.Continuous.LimPID PID_prim_cons(
        controllerType=controllerType,
        k=k_prim_cons,
        Ti=Ti_prim_cons,
        Td=Td_prim_cons,
        yMax=1,
        yMin=0.25,
        initType=initType,
        y_start=PID_prim_cons.yMax)
        annotation (Placement(transformation(extent={{-30,-32},{-10,-12}})));
      Modelica.Blocks.Continuous.LimPID PID_sec_cons(
        controllerType=controllerType,
        k=k_sec_cons,
        Ti=Ti_sec_cons,
        Td=Td_sec_cons,
        yMax=V_dot_sec_max,
        yMin=2,
        initType=initType,
        y_start=PID_sec_cons.yMax)
        annotation (Placement(transformation(extent={{-34,28},{-14,48}})));
      Modelica.Blocks.Continuous.LimPID PID_prim_prod(
        controllerType=controllerType,
        k=k_prim_prod,
        Ti=Ti_prim_prod,
        Td=Td_prim_prod,
        yMax=1,
        yMin=0.25,
        initType=initType,
        y_start=PID_prim_prod.yMax) annotation (Placement(transformation(extent={{12,-30},
                {32,-10}})));
      Modelica.Blocks.Continuous.LimPID PID_sec_prod(
        controllerType=controllerType,
        k=k_sec_prod,
        Ti=Ti_sec_prod,
        Td=Td_sec_prod,
        yMax=V_dot_sec_max,
        yMin=2,
        initType=initType,
        y_start=PID_sec_prod.yMax) annotation (Placement(transformation(extent={{10,28},
                {30,48}})));
      Real T_sec_set(unit="K", displayUnit="degC")
         "Temperature on the secondary side" annotation (Placement(transformation(
            extent={{-20,-20},{20,20}},
            rotation=0,
            origin={80,100})));
      Real V_dot_sec_set(unit="l/min", displayUnit=
           "l/min") "volume flow rate setpoint on the secondary side" annotation (Placement(transformation(
            extent={{-20,-20},{20,20}},
            rotation=0,
            origin={80,60})));
      Modelica.Blocks.Interfaces.RealInput T_sec_in_is(unit="K", displayUnit="degC")
        "Temperature on the secondary side" annotation (Placement(transformation(
            extent={{-20,-20},{20,20}},
            rotation=270,
            origin={60,190})));

    equation

      // assign inputs
      T_prim_hot   = states[1];
      T_prim_cold  = states[2];
      T_sec_hot    = states[3];
      T_sec_cold   = states[4];
      V_dot_prim   = states[5];
      V_dot_sec    = states[6];
      Q_dot_is     = states[7];
      Delta_p_prim = states[8];

      Delta_T_prim      = T_prim_hot -T_prim_cold;
      Delta_T_sec       = T_sec_hot  -T_sec_cold;

      beta_prim_prod = 1 - alpha_prim_prod;
      beta_sec_prod  = 1 - alpha_sec_prod;
      beta_prim_cons = 1 - alpha_prim_cons;
      beta_sec_cons  = 1 - alpha_sec_cons;

      // determine easy static values that just depend on prosumer mode
      // determine inputs for the four PIDs
      // four PIDs in order to be able to have different gains for each situation

      if  Q_dot_set <= 0-tol then // consumption mode
        prosumer_mode = -1;
      elseif Q_dot_set >= 0+tol then // production mode
        prosumer_mode = +1;
      else // idle mode
        prosumer_mode = 0;
      end if;

      Q_dot_is_use = abs(Q_dot_is);
      Q_dot_set_use = abs(Q_dot_set);

      if prosumer_mode == -1 then // consumption mode
        pi_set = 1;
        mu_set = -1;
        T_prim_relev_des = T_sec_hot_des;
        T_prim_relev_is = T_sec_hot;
        T_sec_relev_des = DeltaT_prim_des;
        T_sec_relev_is = T_prim_hot-T_prim_cold;

        PIDin_prim_cons_is_weighted    = alpha_prim_cons*Q_dot_is_use/Delta_Qdot_norm + beta_prim_cons*T_prim_relev_is/Delta_T_norm;
        PIDin_prim_cons_des_weighted   = alpha_prim_cons*Q_dot_set_use/Delta_Qdot_norm + beta_prim_cons*T_prim_relev_des/Delta_T_norm;
        PIDin_prim_prod_is_weighted    = 0;
        PIDin_prim_prod_des_weighted   = 0;
        PIDin_sec_cons_is_weighted     = alpha_sec_cons*Q_dot_is_use/Delta_Qdot_norm + beta_sec_cons*T_sec_relev_is/Delta_T_norm;
        PIDin_sec_cons_des_weighted    = alpha_sec_cons*Q_dot_set_use/Delta_Qdot_norm + beta_sec_cons*T_sec_relev_des/Delta_T_norm;
        PIDin_sec_prod_is_weighted     = 0;
        PIDin_sec_prod_des_weighted    = 0;

        error_prim_weighted            = PIDin_prim_cons_des_weighted - PIDin_prim_cons_is_weighted;
        error_sec_weighted             = PIDin_sec_cons_des_weighted - PIDin_sec_cons_is_weighted;

        error_T_prim_abs               = DeltaT_prim_des - Delta_T_prim;
        error_T_sec_abs                = T_sec_hot_des - T_sec_hot;

        error_T_high_prio_abs          = T_sec_hot_des - T_sec_hot; // T_sec_hot
        error_T_low_prio_abs           = DeltaT_prim_des - Delta_T_prim; // Delta_T_prim

      elseif prosumer_mode == 1 then // production mode
        pi_set = 1;
        mu_set = 1;
        T_prim_relev_des = T_prim_hot_des;
        T_prim_relev_is = T_prim_hot;
        T_sec_relev_des = DeltaT_sec_des;
        T_sec_relev_is = T_sec_hot-T_sec_cold;

        PIDin_prim_cons_is_weighted    = 0;
        PIDin_prim_cons_des_weighted   = 0;
        PIDin_prim_prod_is_weighted    = alpha_prim_prod*Q_dot_is_use/Delta_Qdot_norm + beta_prim_prod*(-1)*T_prim_relev_is/Delta_T_norm;
        PIDin_prim_prod_des_weighted   = alpha_prim_prod*Q_dot_set_use/Delta_Qdot_norm + beta_prim_prod*(-1)*T_prim_relev_des/Delta_T_norm;
        PIDin_sec_cons_is_weighted     = 0;
        PIDin_sec_cons_des_weighted    = 0;
        PIDin_sec_prod_is_weighted     = alpha_sec_prod*Q_dot_is_use/Delta_Qdot_norm + beta_sec_prod*(-1)*T_sec_relev_is/Delta_T_norm;
        PIDin_sec_prod_des_weighted    = alpha_sec_prod*Q_dot_set_use/Delta_Qdot_norm + beta_sec_prod*(-1)*T_sec_relev_des/Delta_T_norm;

        error_prim_weighted            = PIDin_prim_prod_des_weighted - PIDin_prim_prod_is_weighted;
        error_sec_weighted             = PIDin_sec_prod_des_weighted - PIDin_sec_prod_is_weighted;

        error_T_prim_abs               = T_prim_hot_des - T_prim_hot;
        error_T_sec_abs                = DeltaT_sec_des - Delta_T_sec;

        error_T_high_prio_abs          = T_prim_hot_des - T_prim_hot;
        error_T_low_prio_abs           = DeltaT_sec_des - Delta_T_sec;

      else // idle mode
        pi_set = 0;
        mu_set = -1;
        T_prim_relev_des = 0;
        T_prim_relev_is = 0;
        T_sec_relev_des = 0;
        T_sec_relev_is = 0;

        PIDin_prim_cons_is_weighted    = 0;
        PIDin_prim_cons_des_weighted   = 0;
        PIDin_prim_prod_is_weighted    = 0;
        PIDin_prim_prod_des_weighted   = 0;
        PIDin_sec_cons_is_weighted     = 0;
        PIDin_sec_cons_des_weighted    = 0;
        PIDin_sec_prod_is_weighted     = 0;
        PIDin_sec_prod_des_weighted    = 0;

        error_prim_weighted            = 0;
        error_sec_weighted             = 0;

        error_T_prim_abs               = 0;
        error_T_sec_abs                = 0;

        error_T_high_prio_abs          = 0;
        error_T_low_prio_abs           = 0;

      end if;

      // assign PID controller inputs
      PID_prim_cons.u_s    = PIDin_prim_cons_des_weighted;
      PID_prim_cons.u_m    = PIDin_prim_cons_is_weighted;
      PID_prim_prod.u_s    = PIDin_prim_prod_des_weighted;
      PID_prim_prod.u_m    = PIDin_prim_prod_is_weighted;
      PID_sec_cons.u_s     = PIDin_sec_cons_des_weighted;
      PID_sec_cons.u_m     = PIDin_sec_cons_is_weighted;
      PID_sec_prod.u_s     = PIDin_sec_prod_des_weighted;
      PID_sec_prod.u_m     = PIDin_sec_prod_is_weighted;

      // connect secondary side temperature setpoint
      T_sec_set    =  T_sec_in_is;

      // assign PID outputs to controller outputs
      if prosumer_mode == -1 then // consumption mode
        u_set = 0;
        kappa_set =PID_prim_cons.y;
        V_dot_sec_set = PID_sec_cons.y;
      elseif prosumer_mode == 1 then // production mode
        u_set =PID_prim_prod.y;
        kappa_set = 0;
        V_dot_sec_set = PID_sec_prod.y;
      else // idle mode
        V_dot_sec_set = 0;
        u_set = 0;
        kappa_set = 0;
      end if;

      error_Q_abs = Q_dot_set_use - Q_dot_is_use;

      // assign control variables vector
      contr_vars_real[1]   =  T_sec_set;
      contr_vars_real[2]   =  V_dot_sec_set;
      contr_vars_real[3]   =  pi_set;
      contr_vars_real[4]   =  mu_set;
      contr_vars_real[5]   =  u_set;
      contr_vars_real[6]   =  kappa_set;

        annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-120,-160},{
                120,180}}),                                           graphics={
              Text(
              extent={{-70,56},{64,-56}},
              textColor={28,108,200},
              textString="weighted
PID"),     Rectangle(extent={{-120,180},{120,-160}}, lineColor={0,0,0})}),
                                                                       Diagram(
            coordinateSystem(preserveAspectRatio=false, extent={{-120,-160},{120,180}})));
    end PID_Q_T_weighted_mix1;

    model PID_Q_T_weighted_mix2

      import Modelica.Units.SI;
      import T_AbsZeroDegC = Modelica.Constants.T_zero;
      import Modelica.Blocks.Types.Init;
      import Modelica.Blocks.Types.SimpleController;

        // !!!!! parameters !!!!!
      parameter Real Delta_Qdot_norm = 1
          "Heat power value for normalizing the error (deviation) of the transferred heat.
      For alpha=0.5 a deviation of Delta_Qdot_norm in heat transfer is weigthed equal to a deviation of Delta_T_norm in temperature."
          annotation(Dialog(group="Normalizing values"));
      parameter SI.TemperatureDifference Delta_T_norm(min=0) = 3
          "Temperature difference for normalizing the error (deviation) of the temperature.
      For alpha=0.5 a deviation of Delta_T_norm in temperature is weighted equal to a deviation of Delta_Qdot_norm in heat transfer."
          annotation(Dialog(group="Normalizing values"));

      parameter SI.Temperature T_prim_hot_des(min=277)= - T_AbsZeroDegC + 53.5
          "desired temperature supply primary side"
          annotation(Dialog(group="Temperature objectives"));
      parameter SI.Temperature T_sec_hot_des(min=277)= - T_AbsZeroDegC + 50
          "desired temperature supply secondary side"
          annotation(Dialog(group="Temperature objectives"));
      parameter SI.TemperatureDifference DeltaT_prim_des(min=1) =   20
          "desired temperature difference primary side"
          annotation(Dialog(group="Temperature objectives"));
      parameter SI.TemperatureDifference DeltaT_sec_des(min=1) =   20
          "desired temperature difference secondary side"
          annotation(Dialog(group="Temperature objectives"));
      parameter Real V_dot_sec_max(unit="l/min", displayUnit="l/min") = 8.5
        "maximum secondary side volume flow in [l/min]"
        annotation(Dialog(group="General PID settings"));
      parameter Real k_prim_prod = 1.5
        "Proportional gain for controller in [-]"
        annotation(Dialog(group="PID primary side - producer mode - tuning"));
      parameter Real Ti_prim_prod = 8
        "Integral time constant for controller in [s]"
        annotation(Dialog(group="PID primary side - producer mode - tuning"));
      parameter Real Td_prim_prod = 0
        "Derivative time constant for controller in [s]"
        annotation(Dialog(group="PID primary side - producer mode - tuning"));
      parameter Real alpha_prim_prod(min=0, max=1) = 0.666
        "weight for the relevance of the error of the transferred heat in comparison to the error of temperature objectives (sum is one)"
        annotation(Dialog(group="PID primary side - producer mode - tuning"));
      parameter Real k_sec_prod = 1.5
        "Proportional gain for controller in [-]"
        annotation(Dialog(group="PID secondary side - producer mode - tuning"));
      parameter Real Ti_sec_prod = 8
        "Integral time constant for controller in [s]"
        annotation(Dialog(group="PID secondary side - producer mode - tuning"));
      parameter Real Td_sec_prod = 0
        "Derivative time constant for controller in [s]"
        annotation(Dialog(group="PID secondary side - producer mode - tuning"));
      parameter Real alpha_sec_prod(min=0, max=1) = 0.333
        "weight for the relevance of the error of the transferred heat in comparison to the error of temperature objectives (sum is one)"
        annotation(Dialog(group="PID secondary side - producer mode - tuning"));
      parameter Real k_prim_cons = 1.0
        "Proportional gain for controller in [-]"
        annotation(Dialog(group="PID primary side - consumer mode - tuning"));
      parameter Real Ti_prim_cons = 35
        "Integral time constant for controller in [s]"
        annotation(Dialog(group="PID primary side - consumer mode - tuning"));
      parameter Real Td_prim_cons = 0
        "Derivative time constant for controller in [s]"
        annotation(Dialog(group="PID primary side - consumer mode - tuning"));
      parameter Real alpha_prim_cons(min=0, max=1) = 0.666
        "weight for the relevance of the error of the transferred heat in comparison to the error of temperature objectives (sum is one)"
        annotation(Dialog(group="PID primary side - consumer mode - tuning"));
      parameter Real k_sec_cons = 1.5
        "Proportional gain for controller in [-]"
        annotation(Dialog(group="PID secondary side - consumer mode - tuning"));
      parameter Real Ti_sec_cons = 8
        "Integral time constant for controller in [s]"
        annotation(Dialog(group="PID secondary side - consumer mode - tuning"));
      parameter Real Td_sec_cons = 0
        "Derivative time constant for controller in [s]"
        annotation(Dialog(group="PID secondary side - consumer mode - tuning"));
      parameter Real alpha_sec_cons(min=0, max=1) = 0.333
        "weight for the relevance of the error of the transferred heat in comparison to the error of temperature objectives (sum is one)"
        annotation(Dialog(group="PID secondary side - consumer mode - tuning"));
      parameter .Modelica.Blocks.Types.SimpleController controllerType=
             Modelica.Blocks.Types.SimpleController.PID "Type of controller"
             annotation(Dialog(tab="Advanced", group="PIDs general"));
      parameter Init initType = Modelica.Blocks.Types.Init.NoInit
        "Type of initialization (1: no init, 2: steady state, 3: initial state, 4: initial output)"
        annotation(Dialog(tab="Advanced", group="PIDs general"));
      parameter Real tol = 0.1
        "tolerance [kW] for idle mode concerning heat transfer setpoint dotQ"
        annotation(Dialog(tab="Advanced", group="Miscellaneous"));

      // !!!!! variables !!!!!
      Real beta_prim_prod(min=0, max=1)
        "weight for the relevance of the error of the temperature(difference)";
      Real beta_sec_prod(min=0, max=1)
        "weight for the relevance of the error of the temperature(difference)";
      Real beta_prim_cons(min=0, max=1)
        "weight for the relevance of the error of the temperature(difference)";
      Real beta_sec_cons(min=0, max=1)
        "weight for the relevance of the error of the temperature(difference)";
      Integer  prosumer_mode
        "prosumer mode {-1;0;1}";
      Real T_prim_relev_des
          "desired value of relevant temperature (difference)
      for control of primary side";
      Real T_prim_relev_is
          "current value of relevant temperature (difference)
      for control of primary side";
      Real T_sec_relev_des
          "desired value of relevant temperature (difference)
      for control of secondary side";
      Real T_sec_relev_is
          "current value of relevant temperature (difference)
      for control of secondary side";

      Real PIDin_prim_cons_is_weighted
          "weighted input of is-values for PID_prim_cons";
      Real PIDin_prim_cons_des_weighted
          "weighted input of desired values for PID_prim_cons";
      Real PIDin_prim_prod_is_weighted
          "weighted input of is-values for PID_prim_prod";
      Real PIDin_prim_prod_des_weighted
          "weighted input of desired values for PID_prim_prod";
      Real PIDin_sec_cons_is_weighted
          "weighted input of is-values for PID_sec_cons";
      Real PIDin_sec_cons_des_weighted
          "weighted input of desired values for PID_sec_cons";
      Real PIDin_sec_prod_is_weighted
          "weighted input of is-values for PID_sec_prod";
      Real PIDin_sec_prod_des_weighted
          "weighted input of desired values for PID_sec_prod";

      Real error_prim_weighted
          "weighted overall error of primary side controller";

      Real error_sec_weighted
          "weighted overall error of primary side controller";

      Real error_T_prim_abs
          "temperature error of primary side controller";

      Real error_T_sec_abs
          "temperature error of primary side controller";

      Real error_Q_abs
          "temperature error of primary side controller";

      Real error_T_high_prio_abs
          "error of higher prioritized temperature objective";

      Real error_T_low_prio_abs
          "error of lower prioritized temperature objective";

      Real Delta_T_prim
          "weighted overall error of primary side controller";

      Real Delta_T_sec
          "weighted overall error of primary side controller";

      Real Q_dot_is_use;
      Real Q_dot_set_use;

       Real Delta_p_prim(unit="Pa", displayUnit="bar") annotation (
          Placement(transformation(
            extent={{-20,-20},{20,20}},
            rotation=180,
            origin={-80,-140})));

      // !!!!! ports !!!!!

      Modelica.Blocks.Interfaces.RealVectorInput states[8]
        annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));

      Modelica.Blocks.Interfaces.RealVectorOutput contr_vars_real[6]
        annotation (Placement(transformation(extent={{100,-20},{140,20}})));

      Real u_set
        "Normalized velocity of feed-in pump"
          annotation (Placement(transformation(
            extent={{-20,-20},{20,20}},
            rotation=0,
            origin={80,-60}),  iconTransformation(
            extent={{-20,-20},{20,20}},
            rotation=0,
            origin={100,-60})));
      Real kappa_set
        "Normalized flow coefficient for control valve"
         annotation (Placement(transformation(
            extent={{-20,-20},{20,20}},
            rotation=0,
            origin={80,-100}),iconTransformation(
            extent={{-20,-20},{20,20}},
            rotation=0,
            origin={100,-100})));
      Real pi_set
        "Participation" annotation (Placement(transformation(
            extent={{-20,-20},{20,20}},
            rotation=0,
            origin={80,20}),    iconTransformation(
            extent={{-20,-20},{20,20}},
            rotation=0,
            origin={100,20})));
      Real mu_set
        "Operating mode" annotation (Placement(transformation(
            extent={{-20,-20},{20,20}},
            rotation=0,
            origin={80,-20}),  iconTransformation(
            extent={{-20,-20},{20,20}},
            rotation=0,
            origin={100,-20})));
      Real T_sec_hot(
        unit="K",
        displayUnit="degC",
        min=277) "current temperature hot level secondary side"      annotation (Placement(
            transformation(
            extent={{-20,-20},{20,20}},
            rotation=0,
            origin={-80,60})));
      Real T_sec_cold(
        unit="K",
        displayUnit="degC",
        min=277)
               "current temperature cold  level secondary side"    annotation (Placement(
            transformation(
            extent={{-20,-20},{20,20}},
            rotation=0,
            origin={-80,20})));
      Real T_prim_hot(
        unit="K",
        displayUnit="degC",
        min=277)
               "current temperature hot level primary side"        annotation (Placement(
            transformation(
            extent={{-20,-20},{20,20}},
            rotation=0,
            origin={-80,140})));
      Real T_prim_cold(
        unit="K",
        displayUnit="degC",
        min=277)
                "current temperature cold level primary side"       annotation (Placement(
            transformation(
            extent={{-20,-20},{20,20}},
            rotation=0,
            origin={-80,100})));
      Modelica.Blocks.Interfaces.RealInput Q_dot_set(unit="kW", displayUnit="kW")
        "setpoint heat transfer (positive production, negative consumption)"
        annotation (Placement(transformation(extent={{-20,-20},{20,20}},
            rotation=-90,
            origin={-60,188})));
      Real V_dot_prim(unit="l/min", displayUnit="l/min")
        annotation (Placement(transformation(extent={{-100,-40},{-60,0}})));
      Real V_dot_sec(unit="l/min", displayUnit="l/min")
        annotation (Placement(transformation(extent={{-100,-80},{-60,-40}})));

      Real Q_dot_is(unit="kW", displayUnit="kW")
        "currently transferred heat (positive production, negative consumption)"
        annotation (Placement(transformation(extent={{-100,-120},{-60,-80}})));
      Modelica.Blocks.Continuous.LimPID PID_prim_cons(
        controllerType=controllerType,
        k=k_prim_cons,
        Ti=Ti_prim_cons,
        Td=Td_prim_cons,
        yMax=1,
        yMin=0.25,
        initType=initType,
        y_start=PID_prim_cons.yMax)
        annotation (Placement(transformation(extent={{-30,-32},{-10,-12}})));
      Modelica.Blocks.Continuous.LimPID PID_sec_cons(
        controllerType=controllerType,
        k=k_sec_cons,
        Ti=Ti_sec_cons,
        Td=Td_sec_cons,
        yMax=V_dot_sec_max,
        yMin=2,
        initType=initType,
        y_start=PID_sec_cons.yMax)
        annotation (Placement(transformation(extent={{-34,28},{-14,48}})));
      Modelica.Blocks.Continuous.LimPID PID_prim_prod(
        controllerType=controllerType,
        k=k_prim_prod,
        Ti=Ti_prim_prod,
        Td=Td_prim_prod,
        yMax=1,
        yMin=0.25,
        initType=initType,
        y_start=PID_prim_prod.yMax) annotation (Placement(transformation(extent={{12,-30},
                {32,-10}})));
      Modelica.Blocks.Continuous.LimPID PID_sec_prod(
        controllerType=controllerType,
        k=k_sec_prod,
        Ti=Ti_sec_prod,
        Td=Td_sec_prod,
        yMax=V_dot_sec_max,
        yMin=2,
        initType=initType,
        y_start=PID_sec_prod.yMax) annotation (Placement(transformation(extent={{10,28},
                {30,48}})));
      Real T_sec_set(unit="K", displayUnit="degC")
         "Temperature on the secondary side" annotation (Placement(transformation(
            extent={{-20,-20},{20,20}},
            rotation=0,
            origin={80,100})));
      Real V_dot_sec_set(unit="l/min", displayUnit=
           "l/min") "volume flow rate setpoint on the secondary side" annotation (Placement(transformation(
            extent={{-20,-20},{20,20}},
            rotation=0,
            origin={80,60})));
      Modelica.Blocks.Interfaces.RealInput T_sec_in_is(unit="K", displayUnit="degC")
        "Temperature on the secondary side" annotation (Placement(transformation(
            extent={{-20,-20},{20,20}},
            rotation=270,
            origin={60,190})));

    equation

      // assign inputs
      T_prim_hot   = states[1];
      T_prim_cold  = states[2];
      T_sec_hot    = states[3];
      T_sec_cold   = states[4];
      V_dot_prim   = states[5];
      V_dot_sec    = states[6];
      Q_dot_is     = states[7];
      Delta_p_prim = states[8];

      Delta_T_prim      = T_prim_hot -T_prim_cold;
      Delta_T_sec       = T_sec_hot  -T_sec_cold;

      beta_prim_prod = 1 - alpha_prim_prod;
      beta_sec_prod  = 1 - alpha_sec_prod;
      beta_prim_cons = 1 - alpha_prim_cons;
      beta_sec_cons  = 1 - alpha_sec_cons;

      // determine easy static values that just depend on prosumer mode
      // determine inputs for the four PIDs
      // four PIDs in order to be able to have different gains for each situation

      if  Q_dot_set <= 0-tol then // consumption mode
        prosumer_mode = -1;
      elseif Q_dot_set >= 0+tol then // production mode
        prosumer_mode = +1;
      else // idle mode
        prosumer_mode = 0;
      end if;

      Q_dot_is_use = abs(Q_dot_is);
      Q_dot_set_use = abs(Q_dot_set);

      if prosumer_mode == -1 then // consumption mode
        pi_set = 1;
        mu_set = -1;
        T_prim_relev_des = DeltaT_prim_des;
        T_prim_relev_is = T_prim_hot-T_prim_cold;
        T_sec_relev_des = T_sec_hot_des;
        T_sec_relev_is = T_sec_hot;

        PIDin_prim_cons_is_weighted    = alpha_prim_cons*Q_dot_is_use/Delta_Qdot_norm + beta_prim_cons*(-1)*T_prim_relev_is/Delta_T_norm;
        PIDin_prim_cons_des_weighted   = alpha_prim_cons*Q_dot_set_use/Delta_Qdot_norm + beta_prim_cons*(-1)*T_prim_relev_des/Delta_T_norm;
        PIDin_prim_prod_is_weighted    = 0;
        PIDin_prim_prod_des_weighted   = 0;
        PIDin_sec_cons_is_weighted     = alpha_sec_cons*Q_dot_is_use/Delta_Qdot_norm + beta_sec_cons*(-1)*T_sec_relev_is/Delta_T_norm;
        PIDin_sec_cons_des_weighted    = alpha_sec_cons*Q_dot_set_use/Delta_Qdot_norm + beta_sec_cons*(-1)*T_sec_relev_des/Delta_T_norm;
        PIDin_sec_prod_is_weighted     = 0;
        PIDin_sec_prod_des_weighted    = 0;

        error_prim_weighted            = PIDin_prim_cons_des_weighted - PIDin_prim_cons_is_weighted;
        error_sec_weighted             = PIDin_sec_cons_des_weighted - PIDin_sec_cons_is_weighted;

        error_T_prim_abs               = DeltaT_prim_des - Delta_T_prim;
        error_T_sec_abs                = T_sec_hot_des - T_sec_hot;

        error_T_high_prio_abs          = T_sec_hot_des - T_sec_hot;
        error_T_low_prio_abs           = DeltaT_prim_des - Delta_T_prim;

      elseif prosumer_mode == 1 then // production mode
        pi_set = 1;
        mu_set = 1;
        T_prim_relev_des = DeltaT_sec_des;
        T_prim_relev_is = T_sec_hot-T_sec_cold;
        T_sec_relev_des = T_prim_hot_des;
        T_sec_relev_is = T_prim_hot;

        PIDin_prim_cons_is_weighted    = 0;
        PIDin_prim_cons_des_weighted   = 0;
        PIDin_prim_prod_is_weighted    = alpha_prim_prod*Q_dot_is_use/Delta_Qdot_norm + beta_prim_prod*T_prim_relev_is/Delta_T_norm;
        PIDin_prim_prod_des_weighted   = alpha_prim_prod*Q_dot_set_use/Delta_Qdot_norm + beta_prim_prod*T_prim_relev_des/Delta_T_norm;
        PIDin_sec_cons_is_weighted     = 0;
        PIDin_sec_cons_des_weighted    = 0;
        PIDin_sec_prod_is_weighted     = alpha_sec_prod*Q_dot_is_use/Delta_Qdot_norm + beta_sec_prod*T_sec_relev_is/Delta_T_norm;
        PIDin_sec_prod_des_weighted    = alpha_sec_prod*Q_dot_set_use/Delta_Qdot_norm + beta_sec_prod*T_sec_relev_des/Delta_T_norm;

        error_prim_weighted            = PIDin_prim_prod_des_weighted - PIDin_prim_prod_is_weighted;
        error_sec_weighted             = PIDin_sec_prod_des_weighted - PIDin_sec_prod_is_weighted;

        error_T_prim_abs               = T_prim_hot_des - T_prim_hot;
        error_T_sec_abs                = DeltaT_sec_des - Delta_T_sec;

        error_T_high_prio_abs          = T_prim_hot_des - T_prim_hot; // T_pim_hot
        error_T_low_prio_abs           = DeltaT_sec_des - Delta_T_sec; // Deltat_T_sec

      else // idle mode
        pi_set = 0;
        mu_set = -1;
        T_prim_relev_des = 0;
        T_prim_relev_is = 0;
        T_sec_relev_des = 0;
        T_sec_relev_is = 0;

        PIDin_prim_cons_is_weighted    = 0;
        PIDin_prim_cons_des_weighted   = 0;
        PIDin_prim_prod_is_weighted    = 0;
        PIDin_prim_prod_des_weighted   = 0;
        PIDin_sec_cons_is_weighted     = 0;
        PIDin_sec_cons_des_weighted    = 0;
        PIDin_sec_prod_is_weighted     = 0;
        PIDin_sec_prod_des_weighted    = 0;

        error_prim_weighted            = 0;
        error_sec_weighted             = 0;

        error_T_prim_abs               = 0;
        error_T_sec_abs                = 0;

        error_T_high_prio_abs          = 0;
        error_T_low_prio_abs           = 0;

      end if;

      // assign PID controller inputs
      PID_prim_cons.u_s    = PIDin_prim_cons_des_weighted;
      PID_prim_cons.u_m    = PIDin_prim_cons_is_weighted;
      PID_prim_prod.u_s    = PIDin_prim_prod_des_weighted;
      PID_prim_prod.u_m    = PIDin_prim_prod_is_weighted;
      PID_sec_cons.u_s     = PIDin_sec_cons_des_weighted;
      PID_sec_cons.u_m     = PIDin_sec_cons_is_weighted;
      PID_sec_prod.u_s     = PIDin_sec_prod_des_weighted;
      PID_sec_prod.u_m     = PIDin_sec_prod_is_weighted;

      // connect secondary side temperature setpoint
      T_sec_set    =  T_sec_in_is;

      // assign PID outputs to controller outputs
      if prosumer_mode == -1 then // consumption mode
        u_set = 0;
        kappa_set =PID_prim_cons.y;
        V_dot_sec_set = PID_sec_cons.y;
      elseif prosumer_mode == 1 then // production mode
        u_set =PID_prim_prod.y;
        kappa_set = 0;
        V_dot_sec_set = PID_sec_prod.y;
      else // idle mode
        V_dot_sec_set = 0;
        u_set = 0;
        kappa_set = 0;
      end if;

      error_Q_abs = Q_dot_set_use - Q_dot_is_use;

      // assign control variables vector
      contr_vars_real[1]   =  T_sec_set;
      contr_vars_real[2]   =  V_dot_sec_set;
      contr_vars_real[3]   =  pi_set;
      contr_vars_real[4]   =  mu_set;
      contr_vars_real[5]   =  u_set;
      contr_vars_real[6]   =  kappa_set;

        annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-120,-160},{
                120,180}}),                                           graphics={
              Text(
              extent={{-70,56},{64,-56}},
              textColor={28,108,200},
              textString="weighted
PID"),     Rectangle(extent={{-120,180},{120,-160}}, lineColor={0,0,0})}),
                                                                       Diagram(
            coordinateSystem(preserveAspectRatio=false, extent={{-120,-160},{120,180}})));
    end PID_Q_T_weighted_mix2;

    package auxiliary
      block TimeTable_noInterp
        "Generate a discontinuous and non-interpolated signal from a table by using the forward value."

        parameter Real table[:, 2] = fill(0.0, 0, 2)
          "Table matrix (time = first column; e.g., table=[0, 0; 1, 1; 2, 4])"
          annotation(Dialog(groupImage="modelica://Modelica/Resources/Images/Blocks/Sources/TimeTable.png"));
        parameter Modelica.Units.SI.Time timeScale(min=Modelica.Constants.eps)=1
          "Time scale of first table column" annotation (Evaluate=true);
        extends Modelica.Blocks.Interfaces.SignalSource;
        parameter Modelica.Units.SI.Time shiftTime=startTime
          "Shift time of first table column";
      protected
        discrete Real a "Interpolation coefficient a of actual interval (y=a*x+b)";
        discrete Real b "Interpolation coefficient b of actual interval (y=a*x+b)";
        Integer last(start=1) "Last used lower grid index";
        discrete Modelica.Units.SI.Time nextEvent(start=0, fixed=true)
          "Next event instant";
        discrete Real nextEventScaled(start=0, fixed=true)
          "Next scaled event instant";
        Real timeScaled "Scaled time";

        function getInterpolationCoefficients
          "Determine interpolation coefficients and next time event"
          extends Modelica.Icons.Function;
          input Real table[:, 2] "Table for interpolation";
          input Real offset "y-offset";
          input Real startTimeScaled "Scaled time-offset";
          input Real timeScaled "Actual scaled time instant";
          input Integer last "Last used lower grid index";
          input Real TimeEps "Relative epsilon to check for identical time instants";
          input Real shiftTimeScaled "Time shift";
          output Real a "Interpolation coefficient a (y=a*x + b)";
          output Real b "Interpolation coefficient b (y=a*x + b)";
          output Real nextEventScaled "Next scaled event instant";
          output Integer next "New lower grid index";
        protected
          Integer columns=2 "Column to be interpolated";
          Integer ncol=2 "Number of columns to be interpolated";
          Integer nrow=size(table, 1) "Number of table rows";
          Integer next0;
          Real tp;
          Real dt;
        algorithm
          next := last;
          nextEventScaled := timeScaled - TimeEps*abs(timeScaled);
          // in case there are no more time events
          tp := timeScaled + TimeEps*abs(timeScaled);

          if tp < startTimeScaled then
            // First event not yet reached
            nextEventScaled := startTimeScaled;
            a := 0;
            b := offset;
          elseif nrow < 2 then
            // Special action if table has only one row
            a := 0;
            b := offset + table[1, columns];
          else
            tp := tp - shiftTimeScaled;
            // Find next time event instant. Note, that two consecutive time instants
            // in the table may be identical due to a discontinuous point.
            while next < nrow and tp >= table[next, 1] loop
              next := next + 1;
            end while;

            // Define next time event, if last table entry not reached
            if next < nrow then
              nextEventScaled := shiftTimeScaled + table[next, 1];
            end if;

            // Determine interpolation coefficients
            if next == 1 then
              next := 2;
            end if;
            next0 := next - 1;
            dt := table[next, 1] - table[next0, 1];
            if dt <= TimeEps*abs(table[next, 1]) then
              // Interpolation interval is not big enough, use "next" value
              a := 0;
              b := offset + table[next, columns];
            else
              a := table[next0, columns]; // (table[next, columns] - table[next0, columns])/dt;
              b := 0; // offset + table[next0, columns] - a*table[next0, 1];
            end if;
          end if;
          // Take into account shiftTimeScaled "a*(time - shiftTime) + b"
          b := b - a*shiftTimeScaled;
        end getInterpolationCoefficients;
      algorithm
        if noEvent(size(table, 1) > 1) then
          assert(not (table[1, 1] > 0.0 or table[1, 1] < 0.0), "The first point in time has to be set to 0, but is table[1,1] = " + String(table[1, 1]));
        end if;
        when {time >= pre(nextEvent),initial()} then
          (a,b,nextEventScaled,last) := getInterpolationCoefficients(
              table,
              offset,
              startTime/timeScale,
              timeScaled,
              last,
              100*Modelica.Constants.eps,
              shiftTime/timeScale);
          nextEvent := nextEventScaled*timeScale;
        end when;
      equation
        assert(size(table, 1) > 0, "No table values defined.");
        timeScaled = time/timeScale;
        y = a; // a*timeScaled + b;
        annotation (
          Icon(coordinateSystem(
              preserveAspectRatio=true,
              extent={{-100,-100},{100,100}}), graphics={
              Line(points={{-80,68},{-80,-80}}, color={192,192,192}),
              Polygon(
                points={{-80,90},{-88,68},{-72,68},{-80,90}},
                lineColor={192,192,192},
                fillColor={192,192,192},
                fillPattern=FillPattern.Solid),
              Line(points={{-90,-70},{82,-70}}, color={192,192,192}),
              Polygon(
                points={{90,-70},{68,-62},{68,-78},{90,-70}},
                lineColor={192,192,192},
                fillColor={192,192,192},
                fillPattern=FillPattern.Solid),
              Rectangle(
                extent={{-48,70},{2,-50}},
                lineColor={255,255,255},
                fillColor={192,192,192},
                fillPattern=FillPattern.Solid),
              Line(points={{-48,-50},{-48,70},{52,70},{52,-50},{-48,-50},{-48,-20},
                    {52,-20},{52,10},{-48,10},{-48,40},{52,40},{52,70},{2,70},{2,-51}}),
              Text(
                extent={{-150,-150},{150,-110}},
                textString="offset=%offset")}),
              Documentation(info="<html>
<p>
This block generates an output signal by <strong>linear interpolation</strong> in
a table. The time points and function values are stored in a matrix
<strong>table[i,j]</strong>, where the first column table[:,1] contains the
time points and the second column contains the data to be interpolated.
The table interpolation has the following properties:
</p>
<ul>
<li>The interpolation interval is found by a linear search where the interval used in the
    last call is used as start interval.</li>
<li>The time points need to be <strong>monotonically increasing</strong>.</li>
<li><strong>Discontinuities</strong> are allowed, by providing the same
    time point twice in the table.</li>
<li>Values <strong>outside</strong> of the table range, are computed by
    <strong>extrapolation</strong> through the last or first two points of the
    table.</li>
<li>If the table has only <strong>one row</strong>, no interpolation is performed and
    the function value is just returned independently of the actual time instant.</li>
<li>Via parameters <strong>shiftTime</strong> and <strong>offset</strong> the curve defined
    by the table can be shifted both in time and in the ordinate value.
    The time instants stored in the table are therefore <strong>relative</strong>
    to <strong>shiftTime</strong>.</li>
<li>If time &lt; startTime, no interpolation is performed and the offset
    is used as ordinate value for the output.</li>
<li>If the table has more than one row, the first point in time <strong>always</strong> has to be set to <strong>0</strong>, e.g.,
    <strong>table=[1,1;2,2]</strong> is <strong>illegal</strong>. If you want to
    shift the time table in time use the <strong>shiftTime</strong> parameter instead.</li>
<li>The table is implemented in a numerically sound way by
    generating <strong>time events</strong> at interval boundaries.
    This generates continuously differentiable values for the integrator.</li>
<li>Via parameter <strong>timeScale</strong> the first column of the table array can
    be scaled, e.g., if the table array is given in hours (instead of seconds)
    <strong>timeScale</strong> shall be set to 3600.</li>
</ul>
<p>
Example:
</p>
<blockquote><pre>
   table = [0, 0;
            1, 0;
            1, 1;
            2, 4;
            3, 9;
            4, 16];
If, e.g., time = 1.0, the output y =  0.0 (before event), 1.0 (after event)
    e.g., time = 1.5, the output y =  2.5,
    e.g., time = 2.0, the output y =  4.0,
    e.g., time = 5.0, the output y = 23.0 (i.e., extrapolation).
</pre></blockquote>

<p>
<img src=\"modelica://Modelica/Resources/Images/Blocks/Sources/TimeTable.png\"
     alt=\"TimeTable.png\">
</p>

</html>",     revisions="<html>
<h4>Release Notes</h4>
<ul>
<li><em>Oct. 21, 2002</em>
       by Christian Schweiger:<br>
       Corrected interface from
<blockquote><pre>
parameter Real table[:, :]=[0, 0; 1, 1; 2, 4];
</pre></blockquote>
       to
<blockquote><pre>
parameter Real table[:, <strong>2</strong>]=[0, 0; 1, 1; 2, 4];
</pre></blockquote>
       </li>
<li><em>Nov. 7, 1999</em>
       by <a href=\"http://www.robotic.dlr.de/Martin.Otter/\">Martin Otter</a>:<br>
       Realized.</li>
</ul>
</html>"));
      end TimeTable_noInterp;

      model test_procedure

        parameter Real dotQ_low( min = 0) = 8
        "heat transfer setpoint for high heat transfer in test procedure";

        parameter Real dotQ_high( min= 0) = 12
        "heat transfer setpoint for low heat transfer in test procedure";

        parameter Real Tsecin_warm(  min= 0) = 45
        "warm inlet temperature for secondary side";

        parameter Real Tsecin_cold(  min= 0) = 30
         "cold inlet temperature for secondary side";

        parameter Real noise_mu( min = 0) = 0
        "mean value of noise for temperature"
        annotation(Dialog(group="Noise"));

        parameter Real noise_sigma( min = 0) = 5
        "standard deviation of noise for temperature"
        annotation(Dialog(group="Noise"));

        parameter Integer test_procedure[:,3] = [
              0,0,0;
              900,1,6;
              1800,1,6;
              2700,1,6;
              3600,0,0;
              4500,0,0;
              5400,0,0;
              6300,1,6;
              7200,1,9;
              8100,1,6;
              9000,0,0;
              9900,-1,6;
              10800,-1,9;
              11700,-1,6;
              12600,0,0;
              13500,1,9;
              14400,0,0;
              15300,0,0;
              16200,-1,9;
              17100,0,0;
              18000,0,0]
        "time [s], prosumer mode (producer[1], consumer[-1], idle[0]), heat transfer (high[9], low[6])";

        Modelica.Blocks.Sources.CombiTimeTable combiTimeTable(
          tableOnFile=false,
          table=test_procedure,
          smoothness=Modelica.Blocks.Types.Smoothness.ConstantSegments,
          extrapolation=Modelica.Blocks.Types.Extrapolation.Periodic)
          "time [s], prosumer mode (producer[1], consumer[-1], idle[0]), heat transfer (high[9], low[6])"
          annotation (Placement(transformation(extent={{-16,2},{4,22}})));

        Modelica.Blocks.Noise.NormalNoise normalNoise(
          samplePeriod=30,
          mu=noise_mu,
          sigma=noise_sigma)
          annotation (Placement(transformation(extent={{-62,-46},{-42,-26}})));

        Modelica.Blocks.Interfaces.RealOutput dotQ
          annotation (Placement(transformation(extent={{90,30},{110,50}})));
        Modelica.Blocks.Interfaces.RealOutput T
          annotation (Placement(transformation(extent={{90,-30},{110,-10}})));
        inner Modelica.Blocks.Noise.GlobalSeed globalSeed(enableNoise=false,
            fixedSeed=4345)
          annotation (Placement(transformation(extent={{-80,64},{-60,84}})));
      equation

        if combiTimeTable.y[1] == 1 then
          T = Tsecin_warm + 273.15 + normalNoise.y;
            if combiTimeTable.y[2] == 6 then
              dotQ = dotQ_low;
            elseif combiTimeTable.y[2] == 9 then
              dotQ = dotQ_high;
            else
              dotQ = 0;
            end if;
        elseif combiTimeTable.y[1] == -1 then
          T = Tsecin_cold + 273.15 + normalNoise.y;
            if combiTimeTable.y[2] == 6 then
              dotQ = -dotQ_low;
            elseif combiTimeTable.y[2] == 9 then
              dotQ = -dotQ_high;
            else
              dotQ = 0;
            end if;
        else
          T = Tsecin_cold + 273.15;
          dotQ = 0;
        end if;

        annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
              coordinateSystem(preserveAspectRatio=false)));
      end test_procedure;

      function simpleMATLAB_fileConverter
      "Function to import trajectory result files and write them as MatLab compatible .mat f
iles"
      input String filename="filename" "File to be converted" annotation (Dialog(__Dymola_loadSelector(filter="Matlab files (*.mat)",
      caption="Select the results trajectory file")));
      input String varOrigNames[:]={"Time","J1.w","J2.w"} "Variable names/headers in the fil
e in modelica syntax";
      input String varReNames[:]={"Time","Inertia_1_angularVel","Inertia_2_angularVel"}
      "Variable names which will appear in the MATLAB results file";
      input String outputFilename="outputFile.mat";
      protected
      Integer noRows "Number of rows in the trajectory being converted";
      Integer noColumn=12 "Number of columns in the trajectory being converted";
      Real data[:,:] "Data read in from trajectory file";
      Real dataDump[:,:] "Sacrificial dump variable for writeMatrix command";
      Integer i=2 "Loop counter";
      algorithm
      noRows := DymolaCommands.Trajectories.readTrajectorySize(filename);
      data := DymolaCommands.Trajectories.readTrajectory(
      filename,
      varOrigNames,
      noRows);
      data := transpose(data);
      noColumn := size(data, 2);
      while i <= noColumn loop
      dataDump := [data[:, 1],data[:, i]];
      if i == 2 then
      DymolaCommands.MatrixIO.writeMatrix(
      outputFilename,
      varReNames[i],
      dataDump);
      else
      DymolaCommands.MatrixIO.writeMatrix(
      outputFilename,
      varReNames[i],
      dataDump,
      true);
      end if;
      i := i + 1;
      end while;
      annotation (Documentation(info="<html>
<p></p>
</html>"),       uses(DymolaCommands(version="1.4")));
      end simpleMATLAB_fileConverter;
    end auxiliary;

  end Controller_PID_based;

end BidirectionalSubstation;
