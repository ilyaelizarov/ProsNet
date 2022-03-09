within ;
package ProsNet "A library for modeling prosumer-based heat networks"
  extends Modelica.Icons.Package;

annotation (uses(IBPSA(version="3.0.0"), Modelica(version="4.0.0")), Icon(
        graphics={   Bitmap(origin={3,5},     extent={{-77,71},{77,-71}},
          imageSource =                                                                      "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",
          rotation=180)}),
  Documentation(info="<html>
<p>The ProsNet library allows users to simulate prosumer-based heat networks based on a network concept and substation design that is used in an experimental setup in the CoSES research center, at the Technical University of Munich.</p>
</html>"),
  version="1",
  conversion(from(version="", script=
          "modelica://ProsNet/Resources/ConvertFromProsNet_.mos")));
end ProsNet;
