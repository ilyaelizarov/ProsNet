within ProsNet.Under_Development;
package ISGT2024
  model D2c_Licklederer_2prosumers
    new_prosumer_models.heat_transfer_station B1(n=0.5,
      redeclare Fluid.Pumps.Data.Pumps.IMP.NMTSmart25_120to180 feedinPer,
      R_ins_transferpipe=5)                             annotation (Placement(
          transformation(
          extent={{20,-18},{-20,18}},
          rotation=0,
          origin={-48,8})));
    Fluid.Pipes.InsulatedPipe_plug pipe_hot12(R_ins=5,       length=100)
      annotation (Placement(transformation(extent={{-8,-58},{18,-32}})));
    new_prosumer_models.heat_transfer_station B2(n=0.5,
      redeclare Fluid.Pumps.Data.Pumps.IMP.NMTSmart25_120to180 feedinPer,
      R_ins_transferpipe=5)                             annotation (Placement(
          transformation(
          extent={{20,-18},{-20,18}},
          rotation=0,
          origin={50,8})));
    Fluid.Pipes.InsulatedPipe_plug pipe_cold12(R_ins=5,       length=100)
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
    Modelica.Blocks.Sources.RealExpression realExpression(y=273.15) annotation (
        Placement(transformation(
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

    Real Losses;

    Controller_PID_based.auxiliary.TimeTable_noInterp power_set1(table=[0,-5; 600,
          -5; 1200,-5; 1800,-10; 2400,10; 3600,4; 4200,-6; 4800,-10; 5400,-4; 6000,
          6; 6600,10; 7200,-10; 7800,-5; 8400,-5])               annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=-90,
          origin={-66,138})));
    Controller_PID_based.auxiliary.TimeTable_noInterp temp_sec_in1(table=[0,30; 600,
          30; 1200,30; 1800,55; 2400,55; 3600,55; 4200,30; 4800,30; 5400,30; 6000,
          55; 6600,55; 7200,30; 7800,30; 8400,30])                 annotation (
        Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=-90,
          origin={-24,138})));
    Controller_PID_based.auxiliary.TimeTable_noInterp power_set2(table=[0,5; 600,5;
          1200,5; 1800,10; 2400,-10; 3600,-4; 4200,6; 4800,10; 5400,4; 6000,-6; 6600,
          -10; 7200,10; 7800,5; 8400,5])                         annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=-90,
          origin={32,140})));
    Controller_PID_based.auxiliary.TimeTable_noInterp temp_sec_in2(table=[0,55; 600,
          55; 1200,55; 1800,55; 2400,30; 3600,30; 4200,55; 4800,55; 5400,55; 6000,
          30; 6600,30; 7200,55; 7800,55; 8400,55])                 annotation (
        Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=-90,
          origin={74,138})));
    Controller_PID_based.PID_Q_T_weighted_mix2 Ctrl1(
      alpha_prim_prod=0.75,
      alpha_sec_prod=0.25,
      alpha_prim_cons=0.75,
      alpha_sec_cons=0.25)
      annotation (Placement(transformation(extent={{-68,46},{-44,80}})));
    Controller_PID_based.PID_Q_T_weighted_mix2 Ctrl2(
      alpha_prim_prod=0.75,
      alpha_sec_prod=0.25,
      alpha_prim_cons=0.75,
      alpha_sec_cons=0.25)
      annotation (Placement(transformation(extent={{42,46},{66,80}})));
  equation
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
    connect(add.u1, realExpression.y) annotation (Line(points={{-28,109},{-12,109},{-12,
            106},{-8,106},{-8,111.5}}, color={0,0,127}));
    connect(realExpression1.y, add1.u1) annotation (Line(points={{82,109.5},{82,104},{
            68,104},{68,107},{62,107}}, color={0,0,127}));
        Losses = Ctrl1.Q_dot_is + Ctrl2.Q_dot_is;

    connect(temp_sec_in1.y, add.u2) annotation (Line(points={{-24,127},{-24,114},{-34,
            114},{-34,109}}, color={0,0,127}));
    connect(temp_sec_in2.y, add1.u2)
      annotation (Line(points={{74,127},{56,127},{56,107}}, color={0,0,127}));
    connect(power_set1.y, Ctrl1.Q_dot_set) annotation (Line(points={{-66,127},{-66,86},
            {-62,86},{-62,80.8}}, color={0,0,127}));
    connect(add.y, Ctrl1.T_sec_in_is) annotation (Line(points={{-31,97.5},{-31,86},{-50,
            86},{-50,81}}, color={0,0,127}));
    connect(power_set2.y, Ctrl2.Q_dot_set) annotation (Line(points={{32,129},{42,129},
            {42,80.8},{48,80.8}},color={0,0,127}));
    connect(add1.y, Ctrl2.T_sec_in_is) annotation (Line(points={{59,95.5},{58,95.5},{58,
            86},{60,86},{60,81}}, color={0,0,127}));
    connect(Ctrl2.contr_vars_real, B2.contr_vars_real)
      annotation (Line(points={{66,62},{76,62},{76,8},{70.2,8}}, color={0,0,127}));
    connect(Ctrl2.states, B2.states)
      annotation (Line(points={{42,62},{24,62},{24,8},{30,8}}, color={0,0,127}));
    connect(Ctrl1.contr_vars_real, B1.contr_vars_real)
      annotation (Line(points={{-44,62},{-18,62},{-18,8},{-27.8,8}}, color={0,0,127}));
    connect(Ctrl1.states, B1.states)
      annotation (Line(points={{-68,62},{-74,62},{-74,8},{-68,8}}, color={0,0,127}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-120},{
              200,160}})),                                         Diagram(
          coordinateSystem(preserveAspectRatio=false, extent={{-100,-120},{200,160}}),
                                                       graphics={Rectangle(extent={{-92,
                154},{-2,-20}}, lineColor={28,108,200}),         Rectangle(extent={{6,
                154},{96,-20}}, lineColor={28,108,200})}),
      experiment(
        StopTime=8400,
        Interval=1,
        __Dymola_Algorithm="Dassl"));
  end D2c_Licklederer_2prosumers;
end ISGT2024;
