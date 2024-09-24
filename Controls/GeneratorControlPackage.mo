within ProsNet.Controls;
package GeneratorControlPackage
  model ControlGenerator
   model GeneratorControl
    // Define connectors
    connector RealInput = input Real;
    connector BooleanInput = input Boolean;
    connector BooleanOutput = output Boolean;

    // Define parameters
    parameter Real TStartCHP; // Temperature threshold to start the CHP
    parameter Real TStopCHP; // Temperature threshold to stop the CHP

    // Define input And output connectors

     Modelica.Blocks.Interfaces.RealInput TopTlayer
       annotation (Placement(transformation(extent={{-148,12},{-108,52}})));
     Modelica.Blocks.Interfaces.RealInput LowTlayer
       annotation (Placement(transformation(extent={{-148,-98},{-108,-58}})));
     Modelica.Blocks.Interfaces.BooleanInput UseChp
       annotation (Placement(transformation(extent={{-148,-38},{-108,2}})));
     Modelica.Blocks.Interfaces.BooleanOutput CHPON
       annotation (Placement(transformation(extent={{76,-10},{96,10}})));
    protected
    Boolean CHPon_internal; // Internal variable to hold the CHP state

   algorithm
    // Control logic for turning the CHP on
    if (TopTlayer < TStartCHP and UseChp) then
      CHPon_internal := true;
    end if;

    // Additional logic to turn the CHP off when lower layer temperature is too high
    if (LowTlayer > TStopCHP) then
      CHPon_internal := false;
    end if;

    // Assign the internal CHP state to the output
    CHPON := CHPon_internal;

   end GeneratorControl;

   model ElHeaterControl
    // Define connectors
    connector RealInput = input Real;
    connector BooleanInput = input Boolean;
    connector BooleanOutput = output Boolean;

    // Define parameters
    parameter Real deltaTEHon; // Temperature threshold to start the CHP
    parameter Real Tmax_TS; // Temperature threshold to stop the CHP

    // Define input And output connectors

     Modelica.Blocks.Interfaces.RealInput Tlayer5
       annotation (Placement(transformation(extent={{-148,12},{-108,52}})));
     Modelica.Blocks.Interfaces.BooleanInput Use
       annotation (Placement(transformation(extent={{-148,-38},{-108,2}})));
     Modelica.Blocks.Interfaces.BooleanOutput ON
       annotation (Placement(transformation(extent={{76,-10},{96,10}})));
    protected
    Boolean internal; // Internal variable to hold the CHP state

   algorithm
    // Control logic for turning the CHP on
    if (Tmax_TS - Tlayer5 > deltaTEHon and Use) then
      internal := true;
   else
      internal := false;
    end if;

    // Assign the internal CHP state to the output
    ON := internal;

   end ElHeaterControl;

   model HPControl
    // Define connectors
    connector RealInput = input Real;
    connector BooleanInput = input Boolean;
    connector BooleanOutput = output Boolean;

    // Define parameters
    parameter Real TStart; // Temperature threshold to start the
    parameter Real TStop; // Temperature threshold to stop the

    // Define input And output connectors

     Modelica.Blocks.Interfaces.RealInput TopTlayer
       annotation (Placement(transformation(extent={{-148,12},{-108,52}})));
     Modelica.Blocks.Interfaces.RealInput LowTlayer
       annotation (Placement(transformation(extent={{-148,-98},{-108,-58}})));
     Modelica.Blocks.Interfaces.BooleanInput Use
       annotation (Placement(transformation(extent={{-148,-38},{-108,2}})));
     Modelica.Blocks.Interfaces.BooleanOutput ON
       annotation (Placement(transformation(extent={{76,-10},{96,10}})));
    protected
    Boolean on_internal; // Internal variable to hold the  state

   algorithm
    // Control logic for turning the  on
    if (TopTlayer < TStart and Use) then
      on_internal := true;
    end if;

    // Additional logic to turn the  off when lower layer temperature is too high
    if (LowTlayer > TStop) then
      on_internal := false;
    end if;

    // Assign the internal  state to the output
    ON := on_internal;

   end HPControl;

   model SolarThermalControl
    // Define connectors
    connector RealInput = input Real;
    connector BooleanInput = input Boolean;
    connector BooleanOutput = output Boolean;

    // Define parameters
    parameter Real deltaTonST; // Temperature threshold to start the
    parameter Real TCollectorST; // Temperature threshold to stop the

    // Define input And output connectors

     Modelica.Blocks.Interfaces.RealInput Tlayer5
       annotation (Placement(transformation(extent={{-148,12},{-108,52}})));
     Modelica.Blocks.Interfaces.BooleanInput Use
       annotation (Placement(transformation(extent={{-148,-38},{-108,2}})));
     Modelica.Blocks.Interfaces.BooleanOutput ON
       annotation (Placement(transformation(extent={{76,-10},{96,10}})));
    protected
    Boolean internal; // Internal variable to hold the  state

   algorithm
    // Control logic for turning the on
    if (TCollectorST - Tlayer5 > deltaTonST and Use) then
      internal := true;
   else
      internal := false;
    end if;

    // Assign the internal state to the output
    ON := internal;

   end SolarThermalControl;

   model TESNetworkControl
     // Define connectors
     connector RealInput = input Real;
     connector RealOutput = output Real;

     // Define parameters
     parameter Real TFeedInTop = 343.15;         // Temperature threshold for top layer to start feed-in (K)
     parameter Real TFeedInMiddle = 323.15;      // Temperature threshold for middle layer to start feed-in (K)
     parameter Real TExtractTop = 338.15;        // Temperature threshold for top layer to start extraction (K)
     parameter Real TExtractMiddle = 323.15;     // Temperature threshold for middle layer to start extraction (K)
     parameter Real TExtractBottom = 338.15;     // Temperature threshold for bottom layer to stop extraction (K)
     parameter Real deltaT;                      // Delta T for the criteria (in Kelvin)

     // Define input connectors
     Modelica.Blocks.Interfaces.RealInput Tstorage_top
       annotation (Placement(transformation(extent={{-148,52},{-108,92}})));
     Modelica.Blocks.Interfaces.RealInput Tstorage_middle
       annotation (Placement(transformation(extent={{-148,-8},{-108,32}})));
     Modelica.Blocks.Interfaces.RealInput Tstorage_bottom
       annotation (Placement(transformation(extent={{-148,-68},{-108,-28}})));

     // Define output connector
     Modelica.Blocks.Interfaces.RealOutput P
       annotation (Placement(transformation(extent={{76,-10},{96,10}})));

    protected
     Real P_internal;  // Internal variable to hold the power flow state

   algorithm
     // Reset P_internal to zero initially
     P_internal := 0;

     // Control logic for Feed-In
     if (Tstorage_top > TFeedInTop + deltaT and Tstorage_middle > TFeedInMiddle + deltaT) then
       P_internal := 1;
     end if;

     // Control logic for Extraction
     if (Tstorage_top < TExtractTop or Tstorage_middle < TExtractMiddle) then
       P_internal := -1;
     end if;

     // Ensure no feed-in or extraction if conditions don't match
     if (Tstorage_bottom > TExtractBottom) then
       P_internal := 0;
     end if;

     // Assign the internal power flow state to the output
     P := P_internal;

   end TESNetworkControl;

   model TESNetworkControl2
   // Define connectors
     connector RealInput = input Real;
     connector RealOutput = output Real;

     // Define parameters
     parameter Real TFeedInTop = 343.15;         // Temperature threshold for top layer to start feed-in (K)
     parameter Real TFeedInMiddle = 323.15;      // Temperature threshold for middle layer to start feed-in (K)
     parameter Real TExtractTop = 338.15;        // Temperature threshold for top layer to start extraction (K)
     parameter Real TExtractMiddle = 323.15;     // Temperature threshold for middle layer to start extraction (K)
     parameter Real TExtractBottomHigh = 338.15; // Temperature threshold for bottom layer to stop extraction (K)
     parameter Real TExtractBottomLow = 323.15;  // Alternate temperature threshold for bottom layer to stop extraction (K)
     parameter Real deltaT;                      // Delta T for the criteria (in Kelvin)

     // Define input connectors
     Modelica.Blocks.Interfaces.RealInput Tstorage_top
       annotation (Placement(transformation(extent={{-148,52},{-108,92}})));
     Modelica.Blocks.Interfaces.RealInput Tstorage_middle
       annotation (Placement(transformation(extent={{-148,-8},{-108,32}})));
     Modelica.Blocks.Interfaces.RealInput Tstorage_bottom
       annotation (Placement(transformation(extent={{-148,-68},{-108,-28}})));

     // Define output connector
     Modelica.Blocks.Interfaces.RealOutput P
       annotation (Placement(transformation(extent={{76,-10},{96,10}})));

    protected
     Real P_internal;  // Internal variable to hold the power flow state

   algorithm
     // Reset P_internal to zero initially
     P_internal := 0;

     // Control logic for Feed-In
     if (Tstorage_top > TFeedInTop + deltaT and Tstorage_middle > TFeedInMiddle + deltaT) then
       P_internal := 1;
     elseif (Tstorage_top < TFeedInTop or Tstorage_middle < TFeedInMiddle) then
       P_internal := 0;
     end if;

     // Control logic for Extraction
     if (Tstorage_top < TExtractTop or Tstorage_middle < TExtractMiddle) then
       P_internal := -1;
     elseif (Tstorage_bottom > TExtractBottomHigh or Tstorage_bottom > TExtractBottomLow) then
       P_internal := 0;
     end if;

     // Assign the internal power flow state to the output
     P := P_internal;

   end TESNetworkControl2;

   model TESNetworkControl3
     // Define connectors
     connector RealInput = input Real;
     connector RealOutput = output Real;

     // Define parameters (Temperatures in Kelvin)
     parameter Real TFeedInTop = 343.15;
     parameter Real TFeedInMiddle = 323.15;
     parameter Real TExtractTop = 338.15;
     parameter Real TExtractMiddle = 323.15;
     parameter Real TExtractBottom = 338.15;
     parameter Real deltaT;
     // Define input connectors
     Modelica.Blocks.Interfaces.RealInput Tstorage_top
       annotation (Placement(transformation(extent={{-148,52},{-108,92}})));
     Modelica.Blocks.Interfaces.RealInput Tstorage_middle
       annotation (Placement(transformation(extent={{-148,-8},{-108,32}})));
     Modelica.Blocks.Interfaces.RealInput Tstorage_bottom
       annotation (Placement(transformation(extent={{-148,-68},{-108,-28}})));

     // Define output connectors
     Modelica.Blocks.Interfaces.RealOutput P
       annotation (Placement(transformation(extent={{78,-44},{98,-24}})));
     Modelica.Blocks.Interfaces.RealOutput Tout
       annotation (Placement(transformation(extent={{78,52},{98,72}})));

    protected
     Real P_internal;  // Internal variable to hold the power flow state
     Real Tout_internal; // Internal variable to hold the output temperature

   algorithm
     // Reset P_internal and Tout_internal to zero initially
     P_internal := 0;
     Tout_internal := 323;

     // Control logic for Feed-In
     if (Tstorage_top > TFeedInTop + deltaT and Tstorage_middle > TFeedInMiddle + deltaT) then
       P_internal := 1;
       Tout_internal := 65 + 273.15;  // Set Tout to 65°C in Kelvin during feed-in
     end if;

     // Control logic for Extraction
     if (Tstorage_top < TExtractTop or Tstorage_middle < TExtractMiddle) then
       P_internal := -1;
       Tout_internal := 40 + 273.15;  // Set Tout to 40°C in Kelvin during extraction
     end if;

     // Ensure no feed-in or extraction if conditions don't match
     if (Tstorage_bottom > TExtractBottom) then
       P_internal := 0;
       Tout_internal := 323;  // Set Tout to 50 when neither feed-in nor extraction
     end if;

     // Assign the internal power flow state to the output
     P := P_internal;
     Tout := Tout_internal;

   end TESNetworkControl3;

   model HeatFlowRate


     // Output connector


     // Parameter for specific heat capacity
     parameter Real Cp = 4186 "Specific heat capacity (in J/kg·K)"; // Default is for water

     Modelica.Blocks.Interfaces.RealInput Thot "C"
       annotation (Placement(transformation(extent={{-154,24},{-114,64}})));
     Modelica.Blocks.Interfaces.RealInput Tcold "C"
       annotation (Placement(transformation(extent={{-156,-30},{-116,10}})));
     Modelica.Blocks.Interfaces.RealInput mdot "kg per sec"
       annotation (Placement(transformation(extent={{-156,-78},{-116,-38}})));
     Modelica.Blocks.Interfaces.RealOutput Qdot "Watt"
       annotation (Placement(transformation(extent={{62,-4},{82,16}})));
    protected
     Real deltaT; // Temperature difference (in K)

   equation
     // Calculate temperature difference
     deltaT = Thot - Tcold;

     // Calculate heat flow rate
     Qdot = Cp * mdot * deltaT/ 1000;

     // Ensure temperatures are in degrees Celsius but temperature difference (deltaT) is in Kelvin
     // No need to convert since deltaT will be the same in K or °C

      annotation (Icon(graphics={Text(
              extent={{-84,58},{50,-42}},
              textColor={28,108,200},
              textString="Qdot")}));
   end HeatFlowRate;

   model TESNetworkControl3_scenarioC
     // Define connectors
     connector RealInput = input Real;
     connector RealOutput = output Real;

     // Define parameters (Temperatures in Kelvin)
     parameter Real TFeedInTop = 343.15;
     parameter Real TFeedInMiddle = 323.15;
     parameter Real TExtractTop = 338.15;
     parameter Real TExtractMiddle = 323.15;
     parameter Real TExtractBottom = 338.15;
     parameter Real deltaT;

     // Define input connectors
     Modelica.Blocks.Interfaces.RealInput Tstorage_top
       annotation (Placement(transformation(extent={{-148,52},{-108,92}})));
     Modelica.Blocks.Interfaces.RealInput Tstorage_middle
       annotation (Placement(transformation(extent={{-148,-8},{-108,32}})));
     Modelica.Blocks.Interfaces.RealInput Tstorage_bottom
       annotation (Placement(transformation(extent={{-148,-68},{-108,-28}})));

     // Define output connectors
     Modelica.Blocks.Interfaces.RealOutput P
       annotation (Placement(transformation(extent={{78,-44},{98,-24}})));
     Modelica.Blocks.Interfaces.RealOutput Tout
       annotation (Placement(transformation(extent={{78,52},{98,72}})));
     Modelica.Blocks.Interfaces.RealOutput ToutSetPrime
       annotation (Placement(transformation(extent={{78,-100},{98,-80}})));  // New output for ToutSetPrime

    protected
     Real P_internal;  // Internal variable to hold the power flow state
     Real Tout_internal; // Internal variable to hold the output temperature
     Real ToutSetPrime_internal; // Internal variable to hold the ToutSetPrime state

   algorithm
     // Reset P_internal, Tout_internal, and ToutSetPrime_internal to zero initially
     P_internal := 0;
     Tout_internal := 323;
     ToutSetPrime_internal := 323;

     // Control logic for Feed-In
     if (Tstorage_top > TFeedInTop + deltaT and Tstorage_middle > TFeedInMiddle + deltaT) then
       P_internal := 1;
       Tout_internal := 65 + 273.15;  // Set Tout to 65°C in Kelvin during feed-in
       ToutSetPrime_internal := 40 + 273.15;  // Set ToutSetPrime to 40°C in Kelvin when P_internal = 1
     end if;

     // Control logic for Extraction
     if (Tstorage_top < TExtractTop or Tstorage_middle < TExtractMiddle) then
       P_internal := -1;
       Tout_internal := 40 + 273.15;  // Set Tout to 40°C in Kelvin during extraction
       ToutSetPrime_internal := 65 + 273.15;  // Set ToutSetPrime to 65°C in Kelvin when P_internal = -1
     end if;

     // Ensure no feed-in or extraction if conditions don't match
     if (Tstorage_bottom > TExtractBottom) then
       P_internal := 0;
       Tout_internal := 323;  // Set Tout to 50°C in Kelvin when neither feed-in nor extraction
       ToutSetPrime_internal := 323;  // Set ToutSetPrime to default value (50°C in Kelvin)
     end if;

     // Assign the internal states to the outputs
     P := P_internal;
     Tout := Tout_internal;
     ToutSetPrime := ToutSetPrime_internal;

   end TESNetworkControl3_scenarioC;
   annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  end ControlGenerator;
end GeneratorControlPackage;
