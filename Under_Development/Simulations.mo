within ProsNet.Under_Development;
package Simulations
  model Speer_IEWT2023 "Producer and Consumer with Controller"
    Real Losses;

    Under_Development.new_prosumer_models.heat_transfer_station PROSUMER1(
      Q_flow_nominal=25273*1.2,
      T_a1_nominal=338.15,
      T_a2_nominal=318.15,
      m_flow_nominal_1=0.41,
      dp1_nominal=20000,
      m_flow_nominal_2=0.41,
      dp2_nominal=20000,
      redeclare Fluid.Pumps.Data.Pumps_FSP.GrundfosCR17AAAEHQQE feedinPer,
      energyDynamics_feedPump=Modelica.Fluid.Types.Dynamics.DynamicFreeInitial,
      tau_feedPump=3,
      use_inputFilter_feedPump=true,
      riseTime_feedPump=35,
      init_feedPump=Modelica.Blocks.Types.Init.InitialOutput,
      Kv_conVal=1.6,
      use_inputFilter_conVal=true,
      riseTime_conVal=35,
      init_conVal=Modelica.Blocks.Types.Init.InitialOutput,
      Kv_cheVal=11,
      length_transfer_pipe_tot=20,
      zeta_transferstation=14.12,
      d_transferpipe=0.0273,
      R_ins_transferpipe=1000,
      ambient_temperature=system.T_ambient,
      energyDynamics_pumpsSec=Modelica.Fluid.Types.Dynamics.DynamicFreeInitial,
      tau_pumpsSec=3,
      use_inputFilter_pumpsSec=true,
      riseTime_pumpsSec=35,
      tau_cv=10,
      T_start_cv=333.15,
      valve_prim_cons(kFixed=0),
      heat_exchanger(T_a1_nominal=338.15, T_a2_nominal=318.15))
      annotation (Placement(transformation(extent={{-190,-10},{-138,42}})));

    Under_Development.new_prosumer_models.heat_transfer_station PROSUMER2(
      Q_flow_nominal=21508*1.2,
      T_a1_nominal=338.15,
      T_a2_nominal=318.15,
      m_flow_nominal_1=0.35,
      dp1_nominal=20000,
      m_flow_nominal_2=0.35,
      dp2_nominal=20000,
      redeclare Fluid.Pumps.Data.Pumps_FSP.GrundfosCR17AAAEHQQE feedinPer,
      energyDynamics_feedPump=Modelica.Fluid.Types.Dynamics.DynamicFreeInitial,
      tau_feedPump=3,
      use_inputFilter_feedPump=true,
      riseTime_feedPump=35,
      init_feedPump=Modelica.Blocks.Types.Init.InitialOutput,
      Kv_conVal=1.6,
      use_inputFilter_conVal=true,
      riseTime_conVal=35,
      init_conVal=Modelica.Blocks.Types.Init.InitialOutput,
      Kv_cheVal=11,
      length_transfer_pipe_tot=20,
      zeta_transferstation=14.15,
      d_transferpipe=0.0273,
      R_ins_transferpipe=1000,
      ambient_temperature=system.T_ambient,
      energyDynamics_pumpsSec=Modelica.Fluid.Types.Dynamics.DynamicFreeInitial,
      tau_pumpsSec=3,
      use_inputFilter_pumpsSec=true,
      riseTime_pumpsSec=35,
      energyDynamics_cv=Modelica.Fluid.Types.Dynamics.SteadyState,
      tau_cv=10,
      T_start_cv=333.15,
      heat_exchanger(T_a1_nominal=PROSUMER2.T_a1_nominal, T_a2_nominal=
            PROSUMER2.T_a2_nominal))
      annotation (Placement(transformation(extent={{-12,2},{-64,56}})));

    inner Modelica.Fluid.System system(T_ambient=285.15)
      annotation (Placement(transformation(extent={{70,-188},{126,-162}})));
    Fluid.Pipes.InsulatedPipe Pipe_ID_2_pipe_hot_12(
      allowFlowReversal=true,
      T_amb=system.T_ambient,
      R_ins=1000,
      length=40,
      diameter=0.0273,
      zeta=3.6,
      energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
      T_start=313.15)
      annotation (Placement(transformation(extent={{-132,-110},{-96,-74}})));
    Fluid.Pipes.InsulatedPipe Pipe_ID_2_pipe_cold_12(
      allowFlowReversal=true,
      T_amb=system.T_ambient,
      R_ins=1000,
      length=40,
      diameter=0.0273,
      zeta=3.6,
      energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
      T_start=313.15)
      annotation (Placement(transformation(extent={{-98,-66},{-134,-30}})));
    Fluid.Sources.Boundary_pT bou(
      T=330.65,                   nPorts=1, redeclare final package Medium =
          Media.Water)
      annotation (Placement(transformation(extent={{-2,-198},{-46,-154}})));
    Under_Development.Controller_PID_based.PID_Q_T_weighted Controller_1(
      Delta_Qdot_norm=1,
      Delta_T_norm=3,
      T_prim_hot_des=340.15,
      T_sec_hot_des=333.15,
      DeltaT_prim_des=15,
      DeltaT_sec_des=15,
      V_dot_sec_max=29.02,
      k_prim_prod=1.5,
      Ti_prim_prod=35,
      Td_prim_prod=0,
      alpha_prim_prod=0.85,
      k_sec_prod=1.5,
      Ti_sec_prod=35,
      Td_sec_prod=0,
      alpha_sec_prod=0.85,
      k_prim_cons=1.5,
      Ti_prim_cons=35,
      Td_prim_cons=0,
      alpha_prim_cons=0.9,
      k_sec_cons=1.5,
      Ti_sec_cons=35,
      Td_sec_cons=0,
      alpha_sec_cons=0.3,
      controllerType=Modelica.Blocks.Types.SimpleController.PI,
      initType=Modelica.Blocks.Types.Init.InitialOutput)
      annotation (Placement(transformation(extent={{-268,-4},{-226,44}})));
    Under_Development.Controller_PID_based.auxiliary.TimeTable_noInterp T_sec_in_array_1(table=[0,
          341.15; 900,341.15; 1800,341.15; 2700,341.15; 3600,341.15; 4500,
          341.15; 5400,341.15; 6300,341.15; 7200,341.15; 8100,318.15; 9000,
          318.15; 9900,318.15; 10800,318.15; 11700,318.15; 12600,318.15; 13500,
          318.15; 14400,318.15; 15300,318.15; 16200,318.15; 17100,318.15; 18000,
          318.15; 18900,341.15; 19800,341.15; 20700,341.15; 21600,341.15; 22500,
          341.15; 23400,341.15; 24300,341.15; 25200,341.15; 26100,341.15; 27000,
          341.15; 27900,341.15; 28800,341.15; 29700,318.15; 30600,318.15; 31500,
          318.15; 32400,318.15; 33300,318.15])
      annotation (Placement(transformation(extent={{-400,32},{-380,52}})));
    Under_Development.Controller_PID_based.PID_Q_T_weighted Controller_2(
      Delta_Qdot_norm=1,
      Delta_T_norm=3,
      T_prim_hot_des=340.15,
      T_sec_hot_des=333.15,
      DeltaT_prim_des=15,
      DeltaT_sec_des=15,
      V_dot_sec_max=24.7,
      k_prim_prod=1.5,
      Ti_prim_prod=35,
      Td_prim_prod=0,
      alpha_prim_prod=0.85,
      k_sec_prod=1.5,
      Ti_sec_prod=35,
      Td_sec_prod=0,
      alpha_sec_prod=0.85,
      k_prim_cons=1.5,
      Ti_prim_cons=35,
      Td_prim_cons=0,
      alpha_prim_cons=0.9,
      k_sec_cons=1.5,
      Ti_sec_cons=35,
      Td_sec_cons=0,
      alpha_sec_cons=0.4,
      controllerType=Modelica.Blocks.Types.SimpleController.PI,
      initType=Modelica.Blocks.Types.Init.InitialOutput)
      annotation (Placement(transformation(extent={{64,8},{22,56}})));
    Under_Development.Controller_PID_based.auxiliary.TimeTable_noInterp Q_management_array_1(table=[0,
          12.64; 900,12.64; 1800,12.64; 2700,12.64; 3600,25.27; 4500,25.27;
          5400,25.27; 6300,25.27; 7200,12.64; 8100,-12.64; 9000,-25.27; 9900,-25.27;
          10800,-25.27; 11700,-25.27; 12600,-12.64; 13500,-12.64; 14400,-25.27;
          15300,-25.27; 16200,-25.27; 17100,-25.27; 18000,-12.64; 18900,12.64;
          19800,25.27; 20700,25.27; 21600,25.27; 22500,25.27; 23400,12.64;
          24300,6.32; 25200,12.64; 26100,12.64; 27000,12.64; 27900,12.64; 28800,
          6.32; 29700,-6.32; 30600,-12.64; 31500,-12.64; 32400,-12.64; 33300,-12.64])
      annotation (Placement(transformation(extent={{-316,8},{-296,28}})));
    Under_Development.Controller_PID_based.auxiliary.TimeTable_noInterp Q_management_array_2(table=[0,
          10.75; 900,10.75; 1800,10.75; 2700,10.75; 3600,21.51; 4500,21.51;
          5400,21.51; 6300,21.51; 7200,10.75; 8100,-10.75; 9000,-21.51; 9900,-21.51;
          10800,-21.51; 11700,-21.51; 12600,-10.75; 13500,-10.75; 14400,-21.51;
          15300,-21.51; 16200,-21.51; 17100,-21.51; 18000,-10.75; 18900,10.75;
          19800,21.51; 20700,21.51; 21600,21.51; 22500,21.51; 23400,10.75;
          24300,5.38; 25200,10.75; 26100,10.75; 27000,10.75; 27900,10.75; 28800,
          5.38; 29700,-5.38; 30600,-10.75; 31500,-10.75; 32400,-10.75; 33300,-10.75])
      annotation (Placement(transformation(extent={{124,-10},{104,10}})));
    inner Modelica.Blocks.Noise.GlobalSeed globalSeed(enableNoise=false,
        fixedSeed=4345)
      annotation (Placement(transformation(extent={{-126,72},{-106,92}})));
    Modelica.Blocks.Noise.NormalNoise normalNoise(
      samplePeriod=30,
      mu=0,
      sigma=3)
      annotation (Placement(transformation(extent={{-400,70},{-380,90}})));
    Modelica.Blocks.Math.Add add
      annotation (Placement(transformation(extent={{-356,54},{-336,74}})));
    Modelica.Blocks.Noise.NormalNoise normalNoise1(
      samplePeriod=30,
      mu=0,
      sigma=3) annotation (Placement(transformation(extent={{178,54},{158,74}})));
    Modelica.Blocks.Math.Add add1
      annotation (Placement(transformation(extent={{132,32},{112,52}})));
    Under_Development.Controller_PID_based.auxiliary.TimeTable_noInterp T_sec_in_array_2(table=[0,
          341.15; 900,341.15; 1800,341.15; 2700,341.15; 3600,341.15; 4500,
          341.15; 5400,341.15; 6300,341.15; 7200,341.15; 8100,318.15; 9000,
          318.15; 9900,318.15; 10800,318.15; 11700,318.15; 12600,318.15; 13500,
          318.15; 14400,318.15; 15300,318.15; 16200,318.15; 17100,318.15; 18000,
          318.15; 18900,341.15; 19800,341.15; 20700,341.15; 21600,341.15; 22500,
          341.15; 23400,341.15; 24300,341.15; 25200,341.15; 26100,341.15; 27000,
          341.15; 27900,341.15; 28800,341.15; 29700,318.15; 30600,318.15; 31500,
          318.15; 32400,318.15; 33300,318.15])
      annotation (Placement(transformation(extent={{166,-14},{146,6}})));
    Under_Development.new_prosumer_models.heat_transfer_station PROSUMER3(
      Q_flow_nominal=21245*1.2,
      T_a1_nominal=338.15,
      T_a2_nominal=318.15,
      m_flow_nominal_1=0.34,
      dp1_nominal=20000,
      m_flow_nominal_2=0.34,
      dp2_nominal=20000,
      redeclare Fluid.Pumps.Data.Pumps_FSP.GrundfosCR17AAAEHQQE feedinPer,
      energyDynamics_feedPump=Modelica.Fluid.Types.Dynamics.DynamicFreeInitial,
      tau_feedPump=3,
      use_inputFilter_feedPump=true,
      riseTime_feedPump=35,
      init_feedPump=Modelica.Blocks.Types.Init.InitialOutput,
      Kv_conVal=1.6,
      use_inputFilter_conVal=true,
      riseTime_conVal=35,
      init_conVal=Modelica.Blocks.Types.Init.InitialOutput,
      Kv_cheVal=11,
      length_transfer_pipe_tot=20,
      zeta_transferstation=14.15,
      d_transferpipe=0.0273,
      R_ins_transferpipe=1000,
      ambient_temperature=system.T_ambient,
      energyDynamics_pumpsSec=Modelica.Fluid.Types.Dynamics.DynamicFreeInitial,
      tau_pumpsSec=3,
      use_inputFilter_pumpsSec=true,
      riseTime_pumpsSec=35,
      energyDynamics_cv=Modelica.Fluid.Types.Dynamics.SteadyState,
      tau_cv=10,
      T_start_cv=333.15)
      annotation (Placement(transformation(extent={{284,0},{232,54}})));

    Under_Development.Controller_PID_based.PID_Q_T_weighted Controller_3(
      Delta_Qdot_norm=1,
      Delta_T_norm=3,
      T_prim_hot_des=340.15,
      T_sec_hot_des=333.15,
      DeltaT_prim_des=15,
      DeltaT_sec_des=15,
      V_dot_sec_max=24.4,
      k_prim_prod=1.5,
      Ti_prim_prod=35,
      Td_prim_prod=0,
      alpha_prim_prod=0.85,
      k_sec_prod=1.5,
      Ti_sec_prod=35,
      Td_sec_prod=0,
      alpha_sec_prod=0.85,
      k_prim_cons=1.5,
      Ti_prim_cons=35,
      Td_prim_cons=0,
      alpha_prim_cons=0.9,
      k_sec_cons=1.5,
      Ti_sec_cons=35,
      Td_sec_cons=0,
      alpha_sec_cons=0.4,
      controllerType=Modelica.Blocks.Types.SimpleController.PI,
      initType=Modelica.Blocks.Types.Init.InitialOutput)
      annotation (Placement(transformation(extent={{360,8},{318,56}})));
    Modelica.Blocks.Noise.NormalNoise normalNoise2(
      samplePeriod=30,
      mu=0,
      sigma=3) annotation (Placement(transformation(extent={{472,54},{452,74}})));
    Modelica.Blocks.Math.Add add2
      annotation (Placement(transformation(extent={{426,32},{406,52}})));
    Under_Development.Controller_PID_based.auxiliary.TimeTable_noInterp T_sec_in_array_3(table=[0,
          341.15; 900,341.15; 1800,341.15; 2700,341.15; 3600,341.15; 4500,
          341.15; 5400,341.15; 6300,341.15; 7200,341.15; 8100,318.15; 9000,
          318.15; 9900,318.15; 10800,318.15; 11700,318.15; 12600,318.15; 13500,
          341.15; 14400,341.15; 15300,341.15; 16200,341.15; 17100,341.15; 18000,
          341.15; 18900,318.15; 19800,318.15; 20700,318.15; 21600,318.15; 22500,
          318.15; 23400,318.15; 24300,341.15; 25200,341.15; 26100,341.15; 27000,
          341.15; 27900,341.15; 28800,341.15; 29700,318.15; 30600,318.15; 31500,
          318.15; 32400,318.15; 33300,318.15])
      annotation (Placement(transformation(extent={{460,-14},{440,6}})));
    Fluid.Pipes.InsulatedPipe Pipe_ID_4_pipe_cold_23(
      allowFlowReversal=true,
      T_amb=system.T_ambient,
      R_ins=1000,
      length=40,
      diameter=0.036,
      zeta=2.5,
      energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
      T_start=313.15)
      annotation (Placement(transformation(extent={{202,-62},{168,-28}})));
    Fluid.Pipes.InsulatedPipe Pipe_ID_4_pipe_hot_23(
      allowFlowReversal=true,
      T_amb=system.T_ambient,
      R_ins=1000,
      length=40,
      diameter=0.036,
      zeta=2.5,
      energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
      T_start=313.15)
      annotation (Placement(transformation(extent={{168,-106},{202,-72}})));
    Under_Development.Controller_PID_based.auxiliary.TimeTable_noInterp Q_management_array_3(table=[0,
          8.15; 900,8.15; 1800,8.15; 2700,8.15; 3600,16.29; 4500,16.29; 5400,
          16.29; 6300,16.29; 7200,8.15; 8100,-8.15; 9000,-16.29; 9900,-16.29;
          10800,-16.29; 11700,-16.29; 12600,-8.15; 13500,10.62; 14400,21.24;
          15300,21.24; 16200,21.24; 17100,21.24; 18000,10.62; 18900,-10.62;
          19800,-21.24; 20700,-21.24; 21600,-21.24; 22500,-21.24; 23400,-10.62;
          24300,4.07; 25200,8.15; 26100,8.15; 27000,8.15; 27900,8.15; 28800,
          4.07; 29700,-4.07; 30600,-8.15; 31500,-8.15; 32400,-8.15; 33300,-8.15])
      annotation (Placement(transformation(extent={{422,-14},{402,6}})));
    Under_Development.new_prosumer_models.heat_transfer_station PROSUMER4(
      Q_flow_nominal=41827*1.2,
      T_a1_nominal=338.15,
      T_a2_nominal=318.15,
      m_flow_nominal_1=0.68,
      dp1_nominal=20000,
      m_flow_nominal_2=0.68,
      dp2_nominal=20000,
      redeclare Fluid.Pumps.Data.Pumps_FSP.GrundfosCR35AAAEHQQE feedinPer,
      energyDynamics_feedPump=Modelica.Fluid.Types.Dynamics.DynamicFreeInitial,
      tau_feedPump=3,
      use_inputFilter_feedPump=true,
      riseTime_feedPump=35,
      init_feedPump=Modelica.Blocks.Types.Init.InitialOutput,
      Kv_conVal=2.5,
      use_inputFilter_conVal=true,
      riseTime_conVal=35,
      init_conVal=Modelica.Blocks.Types.Init.InitialOutput,
      Kv_cheVal=20,
      length_transfer_pipe_tot=20,
      zeta_transferstation=18.75,
      d_transferpipe=0.036,
      R_ins_transferpipe=1000,
      ambient_temperature=system.T_ambient,
      energyDynamics_pumpsSec=Modelica.Fluid.Types.Dynamics.DynamicFreeInitial,
      tau_pumpsSec=3,
      use_inputFilter_pumpsSec=true,
      riseTime_pumpsSec=35,
      energyDynamics_cv=Modelica.Fluid.Types.Dynamics.SteadyState,
      tau_cv=10,
      T_start_cv=333.15)
      annotation (Placement(transformation(extent={{582,8},{530,62}})));

    Under_Development.Controller_PID_based.PID_Q_T_weighted Controller_4(
      Delta_Qdot_norm=1,
      Delta_T_norm=3,
      T_prim_hot_des=340.15,
      T_sec_hot_des=333.15,
      DeltaT_prim_des=15,
      DeltaT_sec_des=15,
      V_dot_sec_max=48.03,
      k_prim_prod=1.5,
      Ti_prim_prod=35,
      Td_prim_prod=0,
      alpha_prim_prod=0.85,
      k_sec_prod=1.5,
      Ti_sec_prod=35,
      Td_sec_prod=0,
      alpha_sec_prod=0.85,
      k_prim_cons=1.5,
      Ti_prim_cons=35,
      Td_prim_cons=0,
      alpha_prim_cons=0.9,
      k_sec_cons=1.5,
      Ti_sec_cons=35,
      Td_sec_cons=0,
      alpha_sec_cons=0.4,
      controllerType=Modelica.Blocks.Types.SimpleController.PI,
      initType=Modelica.Blocks.Types.Init.InitialOutput)
      annotation (Placement(transformation(extent={{658,10},{616,58}})));
    Modelica.Blocks.Noise.NormalNoise normalNoise3(
      samplePeriod=30,
      mu=0,
      sigma=3) annotation (Placement(transformation(extent={{768,60},{748,80}})));
    Modelica.Blocks.Math.Add add3
      annotation (Placement(transformation(extent={{722,38},{702,58}})));
    Under_Development.Controller_PID_based.auxiliary.TimeTable_noInterp T_sec_in_array_4(table=[0,
          318.15; 900,318.15; 1800,318.15; 2700,318.15; 3600,318.15; 4500,
          318.15; 5400,318.15; 6300,318.15; 7200,318.15; 8100,341.15; 9000,
          341.15; 9900,341.15; 10800,341.15; 11700,341.15; 12600,341.15; 13500,
          341.15; 14400,341.15; 15300,341.15; 16200,341.15; 17100,341.15; 18000,
          341.15; 18900,318.15; 19800,318.15; 20700,318.15; 21600,318.15; 22500,
          318.15; 23400,318.15; 24300,318.15; 25200,318.15; 26100,318.15; 27000,
          318.15; 27900,318.15; 28800,318.15; 29700,341.15; 30600,341.15; 31500,
          341.15; 32400,341.15; 33300,341.15])
      annotation (Placement(transformation(extent={{756,-6},{736,14}})));
    Under_Development.Controller_PID_based.auxiliary.TimeTable_noInterp Q_management_array_4(table=[0,
          -20.91; 900,-20.91; 1800,-20.91; 2700,-20.91; 3600,-41.83; 4500,-41.83;
          5400,-41.83; 6300,-41.83; 7200,-20.91; 8100,20.91; 9000,41.83; 9900,
          41.83; 10800,41.83; 11700,41.83; 12600,20.91; 13500,6.38; 14400,12.77;
          15300,12.77; 16200,12.77; 17100,12.77; 18000,6.38; 18900,-6.38; 19800,
          -12.77; 20700,-12.77; 21600,-12.77; 22500,-12.77; 23400,-6.38; 24300,
          -10.46; 25200,-20.91; 26100,-20.91; 27000,-20.91; 27900,-20.91; 28800,
          -10.46; 29700,10.46; 30600,20.91; 31500,20.91; 32400,20.91; 33300,
          20.91])
      annotation (Placement(transformation(extent={{718,-10},{698,10}})));
    Fluid.Pipes.InsulatedPipe Pipe_ID_6_pipe_hot_34(
      allowFlowReversal=true,
      T_amb=system.T_ambient,
      R_ins=1000,
      length=49.5,
      diameter=0.0419,
      zeta=2.6,
      energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
      T_start=313.15)
      annotation (Placement(transformation(extent={{362,-104},{394,-72}})));
    Fluid.Pipes.InsulatedPipe Pipe_ID_6_pipe_cold_34(
      allowFlowReversal=true,
      T_amb=system.T_ambient,
      R_ins=1000,
      length=49.5,
      diameter=0.0419,
      zeta=2.6,
      energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
      T_start=313.15)
      annotation (Placement(transformation(extent={{394,-60},{360,-26}})));
    Under_Development.new_prosumer_models.heat_transfer_station PROSUMER5(
      Q_flow_nominal=21245*1.2,
      T_a1_nominal=338.15,
      T_a2_nominal=318.15,
      m_flow_nominal_1=0.34,
      dp1_nominal=20000,
      m_flow_nominal_2=0.34,
      dp2_nominal=20000,
      redeclare Fluid.Pumps.Data.Pumps_FSP.GrundfosCR17AAAEHQQE feedinPer,
      energyDynamics_feedPump=Modelica.Fluid.Types.Dynamics.DynamicFreeInitial,
      tau_feedPump=3,
      use_inputFilter_feedPump=true,
      riseTime_feedPump=35,
      init_feedPump=Modelica.Blocks.Types.Init.InitialOutput,
      Kv_conVal=1.6,
      use_inputFilter_conVal=true,
      riseTime_conVal=35,
      init_conVal=Modelica.Blocks.Types.Init.InitialOutput,
      Kv_cheVal=11,
      length_transfer_pipe_tot=20,
      zeta_transferstation=14.15,
      d_transferpipe=0.0273,
      R_ins_transferpipe=1000,
      ambient_temperature=system.T_ambient,
      energyDynamics_pumpsSec=Modelica.Fluid.Types.Dynamics.DynamicFreeInitial,
      tau_pumpsSec=3,
      use_inputFilter_pumpsSec=true,
      riseTime_pumpsSec=35,
      energyDynamics_cv=Modelica.Fluid.Types.Dynamics.SteadyState,
      tau_cv=10,
      T_start_cv=333.15)
      annotation (Placement(transformation(extent={{858,8},{806,62}})));

    Under_Development.Controller_PID_based.PID_Q_T_weighted Controller_5(
      Delta_Qdot_norm=1,
      Delta_T_norm=3,
      T_prim_hot_des=340.15,
      T_sec_hot_des=333.15,
      DeltaT_prim_des=15,
      DeltaT_sec_des=15,
      V_dot_sec_max=24.4,
      k_prim_prod=1.5,
      Ti_prim_prod=35,
      Td_prim_prod=0,
      alpha_prim_prod=0.85,
      k_sec_prod=1.5,
      Ti_sec_prod=35,
      Td_sec_prod=0,
      alpha_sec_prod=0.85,
      k_prim_cons=1.5,
      Ti_prim_cons=35,
      Td_prim_cons=0,
      alpha_prim_cons=0.9,
      k_sec_cons=1.5,
      Ti_sec_cons=35,
      Td_sec_cons=0,
      alpha_sec_cons=0.4,
      controllerType=Modelica.Blocks.Types.SimpleController.PI,
      initType=Modelica.Blocks.Types.Init.InitialOutput)
      annotation (Placement(transformation(extent={{936,20},{894,68}})));
    Modelica.Blocks.Noise.NormalNoise normalNoise4(
      samplePeriod=30,
      mu=0,
      sigma=3) annotation (Placement(transformation(extent={{1046,62},{1026,
              82}})));
    Modelica.Blocks.Math.Add add4
      annotation (Placement(transformation(extent={{1000,40},{980,60}})));
    Under_Development.Controller_PID_based.auxiliary.TimeTable_noInterp T_sec_in_array_5(table=[0,
          318.15; 900,318.15; 1800,318.15; 2700,318.15; 3600,318.15; 4500,
          318.15; 5400,318.15; 6300,318.15; 7200,318.15; 8100,341.15; 9000,
          341.15; 9900,341.15; 10800,341.15; 11700,341.15; 12600,341.15; 13500,
          341.15; 14400,341.15; 15300,341.15; 16200,341.15; 17100,341.15; 18000,
          341.15; 18900,318.15; 19800,318.15; 20700,318.15; 21600,318.15; 22500,
          318.15; 23400,318.15; 24300,318.15; 25200,318.15; 26100,318.15; 27000,
          318.15; 27900,318.15; 28800,318.15; 29700,341.15; 30600,341.15; 31500,
          341.15; 32400,341.15; 33300,341.15])
      annotation (Placement(transformation(extent={{1034,-4},{1014,16}})));
    Under_Development.Controller_PID_based.auxiliary.TimeTable_noInterp Q_management_array_5(table=[0,
          -10.62; 900,-10.62; 1800,-10.62; 2700,-10.62; 3600,-21.24; 4500,-21.24;
          5400,-21.24; 6300,-21.24; 7200,-10.62; 8100,10.62; 9000,21.24; 9900,
          21.24; 10800,21.24; 11700,21.24; 12600,10.62; 13500,6.38; 14400,12.77;
          15300,12.77; 16200,12.77; 17100,12.77; 18000,6.38; 18900,-6.38; 19800,
          -12.77; 20700,-12.77; 21600,-12.77; 22500,-12.77; 23400,-6.38; 24300,
          -5.31; 25200,-10.62; 26100,-10.62; 27000,-10.62; 27900,-10.62; 28800,
          -5.31; 29700,5.31; 30600,10.62; 31500,10.62; 32400,10.62; 33300,10.62])
      annotation (Placement(transformation(extent={{998,-8},{978,12}})));
    Fluid.Pipes.InsulatedPipe Pipe_ID_8_pipe_hot_45(
      allowFlowReversal=true,
      T_amb=system.T_ambient,
      R_ins=1000,
      length=46.5,
      diameter=0.0273,
      zeta=4.3,
      energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
      T_start=313.15)
      annotation (Placement(transformation(extent={{678,-106},{710,-74}})));
    Fluid.Pipes.InsulatedPipe Pipe_ID_8_pipe_cold_45(
      allowFlowReversal=true,
      T_amb=system.T_ambient,
      R_ins=1000,
      length=46.5,
      diameter=0.0273,
      zeta=4.3,
      energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
      T_start=313.15)
      annotation (Placement(transformation(extent={{714,-58},{680,-24}})));
  equation
    Losses =PROSUMER1.Q_dot_trnsf_is + PROSUMER2.Q_dot_trnsf_is;

    connect(Pipe_ID_2_pipe_hot_12.port_a, PROSUMER1.hot_prim) annotation (
        Line(points={{-132,-92},{-182.2,-92},{-182.2,-10.2889}},     color={0,
            127,255}));
    connect(Pipe_ID_2_pipe_hot_12.port_b, PROSUMER2.hot_prim) annotation (
        Line(points={{-96,-92},{-19.8,-92},{-19.8,1.7}},        color={0,127,
            255}));
    connect(Pipe_ID_2_pipe_cold_12.port_a, PROSUMER2.cold_prim) annotation (
        Line(points={{-98,-48},{-52,-48},{-52,-20},{-56.2,-20},{-56.2,2}},
          color={0,127,255}));
    connect(PROSUMER1.cold_prim, Pipe_ID_2_pipe_cold_12.port_b) annotation (
        Line(points={{-145.8,-10},{-145.8,-48},{-134,-48}},      color={0,127,
            255}));
    connect(bou.ports[1], Pipe_ID_2_pipe_hot_12.port_b) annotation (Line(
          points={{-46,-176},{-80,-176},{-80,-92},{-96,-92}}, color={0,127,
            255}));
    connect(Controller_1.T_sec_set, PROSUMER1.T_sec_in_set) annotation (Line(
          points={{-226,44},{-226,52},{-210,52},{-210,36.2222},{-190,36.2222}},
          color={0,0,127}));
    connect(Controller_1.V_dot_sec_set, PROSUMER1.V_dot_sec_set) annotation (
        Line(points={{-226,34.4},{-212,34.4},{-212,30.4444},{-190,30.4444}},
          color={0,0,127}));
    connect(Controller_1.pi_set, PROSUMER1.pi) annotation (Line(points={{-226,
            24.8},{-190,21.7778}}, color={255,127,0}));
    connect(Controller_1.mu_set, PROSUMER1.mu) annotation (Line(points={{-226,
            15.2},{-210,15.2},{-210,16},{-190,16}}, color={255,127,0}));
    connect(PROSUMER1.u_set, Controller_1.u_set) annotation (Line(points={{-190,
            10.2222},{-190,5.6},{-226,5.6}},      color={0,0,127}));
    connect(PROSUMER1.kappa_set, Controller_1.kappa_set) annotation (Line(
          points={{-190,4.44444},{-210,4.44444},{-210,-16},{-226,-16},{-226,-4}},
                  color={0,0,127}));
    connect(PROSUMER1.T_sec_hot, Controller_1.T_sec_hot) annotation (Line(
          points={{-169.2,42},{-169.2,54},{-252,54},{-252,46},{-253.3,46},{
            -253.3,44}},
                  color={0,0,127}));
    connect(PROSUMER1.T_sec_cold, Controller_1.T_sec_cold) annotation (Line(
          points={{-141.467,42},{-192,42},{-192,44},{-240.7,44}},
                                                          color={0,0,127}));
    connect(PROSUMER1.V_dot_sec, Controller_1.V_dot_sec) annotation (Line(
          points={{-155.333,42},{-155.333,58},{-278,58},{-278,34.4},{-268,34.4}},
                    color={0,0,127}));
    connect(PROSUMER1.T_prim_hot, Controller_1.T_prim_hot) annotation (Line(
          points={{-169.2,-10},{-169.2,-18},{-252,-18},{-252,-10},{-253.3,-10},
            {-253.3,-4}}, color={0,0,127}));
    connect(PROSUMER1.T_prim_cold, Controller_1.T_prim_cold) annotation (Line(
          points={{-141.467,-10},{-218,-10},{-218,-12},{-240.7,-12},{-240.7,-4}},
          color={0,0,127}));
    connect(Controller_1.V_dot_prim, PROSUMER1.V_dot_prim) annotation (Line(
          points={{-268,5.6},{-278,5.6},{-278,-22},{-155.333,-22},{-155.333,-10}},
                  color={0,0,127}));
    connect(PROSUMER1.Q_dot_trnsf_is, Controller_1.Qdot_is) annotation (Line(
          points={{-190,-4.22222},{-214,-4.22222},{-214,-24},{-280,-24},{-280,
            15.2},{-268,15.2}}, color={0,0,127}));
    connect(Controller_2.T_sec_set, PROSUMER2.T_sec_in_set) annotation (Line(
          points={{22,56},{22,64},{0,64},{0,50},{-12,50}}, color={0,0,127}));
    connect(Controller_2.V_dot_sec_set, PROSUMER2.V_dot_sec_set) annotation (
        Line(points={{22,46.4},{0,46.4},{0,44},{-12,44}}, color={0,0,127}));
    connect(Controller_2.pi_set, PROSUMER2.pi)
      annotation (Line(points={{22,36.8},{-12,35}}, color={255,127,0}));
    connect(Controller_2.mu_set, PROSUMER2.mu) annotation (Line(points={{22,27.2},
            {0,27.2},{0,29},{-12,29}},       color={255,127,0}));
    connect(Controller_2.u_set, PROSUMER2.u_set) annotation (Line(points={{22,17.6},
            {0,17.6},{0,23},{-12,23}},       color={0,0,127}));
    connect(Controller_2.kappa_set, PROSUMER2.kappa_set) annotation (Line(
          points={{22,8},{14,8},{14,-6},{-2,-6},{-2,14},{-12,14},{-12,17}},
          color={0,0,127}));
    connect(PROSUMER2.T_sec_hot, Controller_2.T_sec_hot) annotation (Line(
          points={{-32.8,56},{-32.8,68},{48,68},{48,58},{49.3,58},{49.3,56}},
          color={0,0,127}));
    connect(PROSUMER2.T_sec_cold, Controller_2.T_sec_cold) annotation (Line(
          points={{-60.5333,56},{-56,56},{-56,66},{36.7,66},{36.7,56}}, color=
           {0,0,127}));
    connect(PROSUMER2.V_dot_sec, Controller_2.V_dot_sec) annotation (Line(
          points={{-46.6667,56},{-46.6667,72},{74,72},{74,46.4},{64,46.4}},
          color={0,0,127}));
    connect(PROSUMER2.T_prim_hot, Controller_2.T_prim_hot) annotation (Line(
          points={{-32.8,2},{-32.8,-10},{48,-10},{48,2},{49.3,2},{49.3,8}},
          color={0,0,127}));
    connect(PROSUMER2.T_prim_cold, Controller_2.T_prim_cold) annotation (Line(
          points={{-60.5333,2},{-60,2},{-60,-16},{36.7,-16},{36.7,8}}, color=
            {0,0,127}));
    connect(PROSUMER2.V_dot_prim, Controller_2.V_dot_prim) annotation (Line(
          points={{-46.6667,2},{-46,2},{-46,-14},{74,-14},{74,17.6},{64,17.6}},
          color={0,0,127}));
    connect(PROSUMER2.Q_dot_trnsf_is, Controller_2.Qdot_is) annotation (Line(
          points={{-12,8},{-4,8},{-4,-18},{76,-18},{76,27.2},{64,27.2}},
          color={0,0,127}));
    connect(Q_management_array_1.y, Controller_1.Qdot_set) annotation (Line(
          points={{-295,18},{-278,18},{-278,24.8},{-268,24.8}}, color={0,0,127}));
    connect(Q_management_array_2.y, Controller_2.Qdot_set) annotation (Line(
          points={{103,0},{84,0},{84,36.8},{64,36.8}}, color={0,0,127}));
    connect(normalNoise.y, add.u1) annotation (Line(points={{-379,80},{-366,80},{
            -366,70},{-358,70}}, color={0,0,127}));
    connect(T_sec_in_array_1.y, add.u2) annotation (Line(points={{-379,42},{-366,
            42},{-366,58},{-358,58}}, color={0,0,127}));
    connect(add.y, Controller_1.T_sec_in_is) annotation (Line(points={{-335,64},{-318,
            64},{-318,45.4118},{-236.5,45.4118}},
                                      color={0,0,127}));
    connect(normalNoise1.y, add1.u1) annotation (Line(points={{157,64},{144,64},{
            144,48},{134,48}}, color={0,0,127}));
    connect(add1.y, Controller_2.T_sec_in_is)
      annotation (Line(points={{111,42},{80,42},{80,57.4118},{32.5,57.4118}},
                                                                  color={0,0,127}));
    connect(T_sec_in_array_2.y, add1.u2) annotation (Line(points={{145,-4},{136,
            -4},{136,26},{142,26},{142,36},{134,36}}, color={0,0,127}));
    connect(Controller_3.T_sec_set, PROSUMER3.T_sec_in_set) annotation (Line(
          points={{318,56},{318,64},{294,64},{294,48},{284,48}}, color={0,0,
            127}));
    connect(Controller_3.V_dot_sec_set, PROSUMER3.V_dot_sec_set) annotation (
        Line(points={{318,46.4},{294,46.4},{294,42},{284,42}}, color={0,0,127}));
    connect(Controller_3.pi_set, PROSUMER3.pi)
      annotation (Line(points={{318,36.8},{284,33}}, color={255,127,0}));
    connect(Controller_3.mu_set, PROSUMER3.mu) annotation (Line(points={{318,
            27.2},{294,27.2},{294,27},{284,27}}, color={255,127,0}));
    connect(Controller_3.u_set, PROSUMER3.u_set) annotation (Line(points={{318,
            17.6},{294,17.6},{294,21},{284,21}},     color={0,0,127}));
    connect(Controller_3.kappa_set, PROSUMER3.kappa_set) annotation (Line(
          points={{318,8},{308,8},{308,-6},{292,-6},{292,14},{284,14},{284,15}},
          color={0,0,127}));
    connect(PROSUMER3.T_sec_hot, Controller_3.T_sec_hot) annotation (Line(
          points={{263.2,54},{263.2,68},{342,68},{342,58},{345.3,58},{345.3,56}},
                  color={0,0,127}));
    connect(PROSUMER3.T_sec_cold, Controller_3.T_sec_cold) annotation (Line(
          points={{235.467,54},{238,54},{238,66},{332.7,66},{332.7,56}},
          color={0,0,127}));
    connect(PROSUMER3.V_dot_sec, Controller_3.V_dot_sec) annotation (Line(
          points={{249.333,54},{249.333,72},{368,72},{368,46.4},{360,46.4}},
          color={0,0,127}));
    connect(PROSUMER3.T_prim_hot, Controller_3.T_prim_hot) annotation (Line(
          points={{263.2,0},{263.2,-10},{342,-10},{342,2},{345.3,2},{345.3,8}},
          color={0,0,127}));
    connect(PROSUMER3.T_prim_cold, Controller_3.T_prim_cold) annotation (Line(
          points={{235.467,0},{234,0},{234,-16},{332.7,-16},{332.7,8}}, color=
           {0,0,127}));
    connect(PROSUMER3.V_dot_prim, Controller_3.V_dot_prim) annotation (Line(
          points={{249.333,0},{248,0},{248,-14},{368,-14},{368,17.6},{360,17.6}},
                    color={0,0,127}));
    connect(PROSUMER3.Q_dot_trnsf_is, Controller_3.Qdot_is) annotation (Line(
          points={{284,6},{290,6},{290,-18},{370,-18},{370,27.2},{360,27.2}},
          color={0,0,127}));
    connect(normalNoise2.y,add2. u1) annotation (Line(points={{451,64},{438,64},{
            438,48},{428,48}}, color={0,0,127}));
    connect(add2.y, Controller_3.T_sec_in_is)
      annotation (Line(points={{405,42},{374,42},{374,57.4118},{328.5,57.4118}},
                                                                     color={0,0,127}));
    connect(T_sec_in_array_3.y, add2.u2) annotation (Line(points={{439,-4},{430,
            -4},{430,26},{436,26},{436,36},{428,36}}, color={0,0,127}));
    connect(Pipe_ID_2_pipe_cold_12.port_a, Pipe_ID_4_pipe_cold_23.port_b)
      annotation (Line(points={{-98,-48},{42,-48},{42,-45},{168,-45}}, color=
            {0,127,255}));
    connect(Pipe_ID_2_pipe_hot_12.port_b, Pipe_ID_4_pipe_hot_23.port_a)
      annotation (Line(points={{-96,-92},{38,-92},{38,-89},{168,-89}}, color=
            {0,127,255}));
    connect(Pipe_ID_4_pipe_cold_23.port_a, PROSUMER3.cold_prim) annotation (
        Line(points={{202,-45},{240,-45},{240,-8},{239.8,-8},{239.8,0}},
          color={0,127,255}));
    connect(PROSUMER3.hot_prim, Pipe_ID_4_pipe_hot_23.port_b) annotation (
        Line(points={{276.2,-0.3},{256,-0.3},{256,-89},{202,-89}},   color={0,
            127,255}));
    connect(Q_management_array_3.y, Controller_3.Qdot_set) annotation (Line(
          points={{401,-4},{376,-4},{376,36.8},{360,36.8}}, color={0,0,127}));
    connect(Controller_4.T_sec_set, PROSUMER4.T_sec_in_set) annotation (Line(
          points={{616,58},{616,70},{590,70},{590,56},{582,56}}, color={0,0,
            127}));
    connect(Controller_4.V_dot_sec_set, PROSUMER4.V_dot_sec_set) annotation (
        Line(points={{616,48.4},{590,48.4},{590,50},{582,50}}, color={0,0,127}));
    connect(Controller_4.pi_set, PROSUMER4.pi)
      annotation (Line(points={{616,38.8},{582,41}}, color={255,127,0}));
    connect(Controller_4.mu_set, PROSUMER4.mu) annotation (Line(points={{616,
            29.2},{590,29.2},{590,35},{582,35}}, color={255,127,0}));
    connect(Controller_4.u_set, PROSUMER4.u_set) annotation (Line(points={{616,
            19.6},{590,19.6},{590,29},{582,29}},     color={0,0,127}));
    connect(Controller_4.kappa_set, PROSUMER4.kappa_set) annotation (Line(
          points={{616,10},{604,10},{604,0},{588,0},{588,20},{582,20},{582,23}},
          color={0,0,127}));
    connect(PROSUMER4.T_sec_hot, Controller_4.T_sec_hot) annotation (Line(
          points={{561.2,62},{561.2,74},{638,74},{638,64},{643.3,64},{643.3,58}},
                  color={0,0,127}));
    connect(PROSUMER4.T_sec_cold, Controller_4.T_sec_cold) annotation (Line(
          points={{533.467,62},{534,62},{534,72},{630.7,72},{630.7,58}},
          color={0,0,127}));
    connect(PROSUMER4.V_dot_sec, Controller_4.V_dot_sec) annotation (Line(
          points={{547.333,62},{547.333,78},{664,78},{664,48.4},{658,48.4}},
          color={0,0,127}));
    connect(PROSUMER4.T_prim_hot, Controller_4.T_prim_hot) annotation (Line(
          points={{561.2,8},{561.2,-4},{638,-4},{638,8},{643.3,8},{643.3,10}},
          color={0,0,127}));
    connect(PROSUMER4.T_prim_cold, Controller_4.T_prim_cold) annotation (Line(
          points={{533.467,8},{530,8},{530,-10},{630.7,-10},{630.7,10}},
          color={0,0,127}));
    connect(PROSUMER4.V_dot_prim, Controller_4.V_dot_prim) annotation (Line(
          points={{547.333,8},{544,8},{544,-8},{664,-8},{664,19.6},{658,19.6}},
          color={0,0,127}));
    connect(PROSUMER4.Q_dot_trnsf_is, Controller_4.Qdot_is) annotation (Line(
          points={{582,14},{586,14},{586,-12},{666,-12},{666,29.2},{658,29.2}},
          color={0,0,127}));
    connect(normalNoise3.y,add3. u1) annotation (Line(points={{747,70},{734,
            70},{734,54},{724,54}},
                               color={0,0,127}));
    connect(add3.y, Controller_4.T_sec_in_is)
      annotation (Line(points={{701,48},{670,48},{670,59.4118},{626.5,59.4118}},
                                                                     color={0,0,127}));
    connect(T_sec_in_array_4.y,add3. u2) annotation (Line(points={{735,4},{726,
            4},{726,32},{732,32},{732,42},{724,42}},  color={0,0,127}));
    connect(Q_management_array_4.y,Controller_4. Qdot_set) annotation (Line(
          points={{697,0},{672,0},{672,38.8},{658,38.8}},   color={0,0,127}));
    connect(Pipe_ID_6_pipe_hot_34.port_a, Pipe_ID_4_pipe_hot_23.port_b)
      annotation (Line(points={{362,-88},{282,-88},{282,-89},{202,-89}},
          color={0,127,255}));
    connect(PROSUMER4.cold_prim, Pipe_ID_6_pipe_cold_34.port_a) annotation (
        Line(points={{537.8,8},{537.2,8},{537.2,-43},{394,-43}},     color={0,
            127,255}));
    connect(Pipe_ID_6_pipe_cold_34.port_b, PROSUMER3.cold_prim) annotation (
        Line(points={{360,-43},{238,-43},{238,-45},{240,-45},{240,-8},{239.8,-8},{239.8,
            0}},               color={0,127,255}));
    connect(PROSUMER4.hot_prim, Pipe_ID_6_pipe_hot_34.port_b) annotation (
        Line(points={{574.2,7.7},{554.866,7.7},{554.866,-88},{394,-88}},
          color={0,127,255}));
    connect(Controller_5.T_sec_set, PROSUMER5.T_sec_in_set) annotation (Line(
          points={{894,68},{894,72},{868,72},{868,56},{858,56}}, color={0,0,
            127}));
    connect(Controller_5.V_dot_sec_set, PROSUMER5.V_dot_sec_set) annotation (
        Line(points={{894,58.4},{868,58.4},{868,50},{858,50}}, color={0,0,127}));
    connect(Controller_5.pi_set, PROSUMER5.pi)
      annotation (Line(points={{894,48.8},{858,41}}, color={255,127,0}));
    connect(Controller_5.mu_set, PROSUMER5.mu) annotation (Line(points={{894,
            39.2},{868,39.2},{868,35},{858,35}}, color={255,127,0}));
    connect(Controller_5.u_set, PROSUMER5.u_set) annotation (Line(points={{894,
            29.6},{868,29.6},{868,29},{858,29}},     color={0,0,127}));
    connect(Controller_5.kappa_set, PROSUMER5.kappa_set) annotation (Line(
          points={{894,20},{882,20},{882,2},{866,2},{866,22},{858,22},{858,23}},
          color={0,0,127}));
    connect(PROSUMER5.T_sec_hot, Controller_5.T_sec_hot) annotation (Line(
          points={{837.2,62},{837.2,76},{916,76},{916,68},{921.3,68}},
                  color={0,0,127}));
    connect(PROSUMER5.T_sec_cold, Controller_5.T_sec_cold) annotation (Line(
          points={{809.467,62},{812,62},{812,74},{908.7,74},{908.7,68}},
          color={0,0,127}));
    connect(PROSUMER5.V_dot_sec, Controller_5.V_dot_sec) annotation (Line(
          points={{823.333,62},{823.333,80},{942,80},{942,58.4},{936,58.4}},
          color={0,0,127}));
    connect(PROSUMER5.T_prim_hot, Controller_5.T_prim_hot) annotation (Line(
          points={{837.2,8},{837.2,-2},{916,-2},{916,10},{921.3,10},{921.3,20}},
          color={0,0,127}));
    connect(PROSUMER5.T_prim_cold, Controller_5.T_prim_cold) annotation (Line(
          points={{809.467,8},{808,8},{808,-8},{908.7,-8},{908.7,20}}, color=
            {0,0,127}));
    connect(PROSUMER5.V_dot_prim, Controller_5.V_dot_prim) annotation (Line(
          points={{823.333,8},{822,8},{822,-6},{942,-6},{942,29.6},{936,29.6}},
          color={0,0,127}));
    connect(PROSUMER5.Q_dot_trnsf_is, Controller_5.Qdot_is) annotation (Line(
          points={{858,14},{864,14},{864,-10},{944,-10},{944,39.2},{936,39.2}},
          color={0,0,127}));
    connect(normalNoise4.y,add4. u1) annotation (Line(points={{1025,72},{1012,
            72},{1012,56},{1002,56}},
                               color={0,0,127}));
    connect(add4.y, Controller_5.T_sec_in_is)
      annotation (Line(points={{979,50},{948,50},{948,69.4118},{904.5,69.4118}},
                                                                     color={0,0,127}));
    connect(T_sec_in_array_5.y,add4. u2) annotation (Line(points={{1013,6},{
            1004,6},{1004,34},{1010,34},{1010,44},{1002,44}},
                                                      color={0,0,127}));
    connect(Q_management_array_5.y,Controller_5. Qdot_set) annotation (Line(
          points={{977,2},{950,2},{950,48.8},{936,48.8}},   color={0,0,127}));
    connect(PROSUMER5.cold_prim, Pipe_ID_8_pipe_cold_45.port_a) annotation (
        Line(points={{813.8,8},{815.2,8},{815.2,-41},{714,-41}},     color={0,
            127,255}));
    connect(Pipe_ID_8_pipe_cold_45.port_b, Pipe_ID_6_pipe_cold_34.port_a)
      annotation (Line(points={{680,-41},{610,-41},{610,-44},{536,-44},{536,-43},
            {394,-43}}, color={0,127,255}));
    connect(Pipe_ID_8_pipe_hot_45.port_b, PROSUMER5.hot_prim) annotation (
        Line(points={{710,-90},{834,-90},{834,7.7},{850.2,7.7}},   color={0,
            127,255}));
    connect(Pipe_ID_8_pipe_hot_45.port_a, Pipe_ID_6_pipe_hot_34.port_b)
      annotation (Line(points={{678,-90},{616,-90},{616,-88},{394,-88}},
          color={0,127,255}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-440,
              -220},{1060,120}})),Diagram(coordinateSystem(preserveAspectRatio=
              false, extent={{-440,-220},{1060,120}}),graphics={
          Text(
            extent={{-142,-24},{-98,-38}},
            textColor={28,108,200},
            textString="cold"),
          Text(
            extent={{-134,-124},{-98,-140}},
            textColor={238,46,47},
            textString="hot"),
          Text(
            extent={{514,116},{594,82}},
            textColor={0,0,0},
            textString="Prosumer4
MF5"),    Text(
            extent={{792,118},{872,84}},
            textColor={0,0,0},
            textString="Prosumer5
SF2"),    Text(
            extent={{268,110},{348,76}},
            textColor={0,0,0},
            textString="Prosumer3
SF4"),    Text(
            extent={{-32,114},{48,80}},
            textColor={0,0,0},
            textString="Prosumer2
SF3"),    Text(
            extent={{-252,102},{-172,68}},
            textColor={0,0,0},
            textString="Prosumer1
SF1"),    Text(
            extent={{170,-118},{206,-134}},
            textColor={238,46,47},
            textString="hot"),
          Text(
            extent={{162,-18},{206,-32}},
            textColor={28,108,200},
            textString="cold"),
          Text(
            extent={{364,-118},{400,-134}},
            textColor={238,46,47},
            textString="hot"),
          Text(
            extent={{356,-18},{400,-32}},
            textColor={28,108,200},
            textString="cold"),
          Text(
            extent={{680,-116},{716,-132}},
            textColor={238,46,47},
            textString="hot"),
          Text(
            extent={{672,-16},{716,-30}},
            textColor={28,108,200},
            textString="cold")}),
      experiment(
        StopTime=34200,
        Interval=10,
        __Dymola_Algorithm="Dassl"),
      __Dymola_Commands(file="Scripts/COSES_PLOT.mos" "COSES_PLOT", file=
            "Scripts/cosesplot.mos" "cosesplot",
        file="Scripts/cosesplot.mos" "cosesplot",
        file="Scripts/cosesplot.mos" "cosesplot",
        file="Scripts/coses_plot_tables.mos" "coses_plot_tables"));
  end Speer_IEWT2023;

  model Licklederer_weiPID_01
    new_prosumer_models.heat_transfer_station B1(n=0.5, redeclare
        Fluid.Pumps.Data.Pumps.IMP.NMTSmart25_120to180 feedinPer)
                                                        annotation (Placement(
          transformation(
          extent={{20,-18},{-20,18}},
          rotation=0,
          origin={-48,8})));
    Controller_PID_based.PID_Q_T_weighted Ctrl1 annotation (Placement(transformation(
          extent={{-12,-17},{12,17}},
          rotation=0,
          origin={-46,73})));
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
    new_prosumer_models.heat_transfer_station B2(n=0.5, redeclare
        Fluid.Pumps.Data.Pumps.IMP.NMTSmart25_120to180 feedinPer)
                                                        annotation (Placement(
          transformation(
          extent={{20,-18},{-20,18}},
          rotation=0,
          origin={50,8})));
    Controller_PID_based.PID_Q_T_weighted Ctrl2 annotation (Placement(transformation(
          extent={{-12,-17},{12,17}},
          rotation=0,
          origin={52,73})));
    Controller_PID_based.auxiliary.TimeTable_noInterp power_set2(table=[0,-4; 900,-4;
          1800,-4; 2700,-4; 3600,4; 4500,10; 5400,-10; 6300,-10])
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
    new_prosumer_models.heat_transfer_station B3(n=0.5, redeclare
        Fluid.Pumps.Data.Pumps.IMP.NMTSmart25_120to180 feedinPer)
                                                        annotation (Placement(
          transformation(
          extent={{20,-18},{-20,18}},
          rotation=0,
          origin={148,8})));
    Controller_PID_based.PID_Q_T_weighted Ctrl3 annotation (Placement(transformation(
          extent={{-12,-17},{12,17}},
          rotation=0,
          origin={150,73})));
    Controller_PID_based.auxiliary.TimeTable_noInterp power_set3(table=[0,-6; 900,-6;
          1800,-6; 2700,-6; 3600,6; 4500,-6; 5400,6; 6300,6])    annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=-90,
          origin={126,134})));
    Controller_PID_based.auxiliary.TimeTable_noInterp temp_sec_in3(table=[0,30; 900,30;
          1800,30; 2700,30; 3600,55; 4500,30; 5400,55; 6300,55])   annotation (
        Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=-90,
          origin={168,134})));
    Fluid.Pipes.InsulatedPipe_plug pipe_hot23
      annotation (Placement(transformation(extent={{88,-58},{114,-32}})));
    Fluid.Pipes.InsulatedPipe_plug pipe_cold23
      annotation (Placement(transformation(extent={{114,-103},{88,-77}})));
    Fluid.Pipes.InsulatedPipe_plug pipe_cold12
      annotation (Placement(transformation(extent={{18,-103},{-8,-77}})));
    Modelica.Fluid.Sources.Boundary_pT boundary(
      redeclare package Medium = Media.Water,
      use_p_in=false,
      T=325.4,
      nPorts=1) annotation (Placement(transformation(extent={{-92,-55},{-72,-35}})));
    inner Modelica.Fluid.System system(T_ambient=285.15)
      annotation (Placement(transformation(extent={{-92,-114},{-72,-94}})));
  equation
    connect(B1.contr_vars_real, Ctrl1.contr_vars_real)
      annotation (Line(points={{-27.8,8},{-20,8},{-20,72},{-34,72}}, color={0,0,127}));
    connect(Ctrl1.states, B1.states)
      annotation (Line(points={{-58,72},{-74,72},{-74,8},{-68,8}}, color={0,0,127}));
    connect(power_set1.y, Ctrl1.Q_dot_set) annotation (Line(points={{-70,123},{-70,118},
            {-52,118},{-52,90.8}}, color={0,0,127}));
    connect(temp_sec_in1.y, Ctrl1.T_sec_in_is) annotation (Line(points={{-28,123},{-28,
            118},{-40,118},{-40,91}}, color={0,0,127}));
    connect(B2.contr_vars_real,Ctrl2. contr_vars_real)
      annotation (Line(points={{70.2,8},{78,8},{78,72},{64,72}},     color={0,0,127}));
    connect(Ctrl2.states,B2. states)
      annotation (Line(points={{40,72},{24,72},{24,8},{30,8}},     color={0,0,127}));
    connect(power_set2.y,Ctrl2. Q_dot_set) annotation (Line(points={{28,123},{28,118},{
            46,118},{46,90.8}},    color={0,0,127}));
    connect(temp_sec_in2.y,Ctrl2. T_sec_in_is) annotation (Line(points={{70,123},{70,
            118},{58,118},{58,91}},   color={0,0,127}));
    connect(B3.contr_vars_real,Ctrl3. contr_vars_real)
      annotation (Line(points={{168.2,8},{176,8},{176,72},{162,72}}, color={0,0,127}));
    connect(Ctrl3.states,B3. states)
      annotation (Line(points={{138,72},{122,72},{122,8},{128,8}}, color={0,0,127}));
    connect(power_set3.y,Ctrl3. Q_dot_set) annotation (Line(points={{126,123},{126,118},
            {144,118},{144,90.8}}, color={0,0,127}));
    connect(temp_sec_in3.y,Ctrl3. T_sec_in_is) annotation (Line(points={{168,123},{168,
            118},{156,118},{156,91}}, color={0,0,127}));
    connect(B1.hot_prim, pipe_hot12.port_a)
      annotation (Line(points={{-34,-10.2},{-34,-45},{-8,-45}}, color={0,127,255}));
    connect(pipe_hot12.port_b, B2.hot_prim)
      annotation (Line(points={{18,-45},{64,-45},{64,-10.2}}, color={0,127,255}));
    connect(pipe_hot23.port_a, B2.hot_prim)
      annotation (Line(points={{88,-45},{64,-45},{64,-10.2}}, color={0,127,255}));
    connect(pipe_hot23.port_b, B3.hot_prim) annotation (Line(points={{114,-45},{138,-45},
            {138,-46},{162,-46},{162,-10.2}}, color={0,127,255}));
    connect(B1.cold_prim, pipe_cold12.port_b)
      annotation (Line(points={{-62,-10},{-62,-90},{-8,-90}}, color={0,127,255}));
    connect(pipe_cold12.port_a, B2.cold_prim)
      annotation (Line(points={{18,-90},{36,-90},{36,-10}}, color={0,127,255}));
    connect(B2.cold_prim, pipe_cold23.port_b)
      annotation (Line(points={{36,-10},{36,-90},{88,-90}}, color={0,127,255}));
    connect(pipe_cold23.port_a, B3.cold_prim) annotation (Line(points={{114,-90},{124,
            -90},{124,-92},{134,-92},{134,-10}}, color={0,127,255}));
    connect(pipe_hot12.port_a, boundary.ports[1])
      annotation (Line(points={{-8,-45},{-72,-45}}, color={0,127,255}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-120},{
              200,160}})),                                         Diagram(
          coordinateSystem(preserveAspectRatio=false, extent={{-100,-120},{200,160}}),
                                                       graphics={Rectangle(extent={{-92,
                154},{-2,-20}}, lineColor={28,108,200}),         Rectangle(extent={{6,
                154},{96,-20}}, lineColor={28,108,200}),         Rectangle(extent={{104,
                154},{194,-20}},lineColor={28,108,200})}),
      experiment(
        StopTime=6300,
        Interval=10,
        __Dymola_Algorithm="Dassl"));
  end Licklederer_weiPID_01;
end Simulations;
