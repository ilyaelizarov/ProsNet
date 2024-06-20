within ProsNet.Utilities;
package Time "Package with models for time"
  extends Modelica.Icons.Package;

  model CalendarTime
    "Computes the unix time stamp and calendar time from the simulation time"
    extends Modelica.Blocks.Icons.DiscreteBlock;
    parameter ProsNet.Utilities.Time.Types.ZeroTime zerTim
      "Enumeration for choosing how reference time (time = 0) should be defined";
    parameter Integer yearRef(min=firstYear, max=lastYear) = 2016
      "Year when time = 0, used if zerTim=Custom"
      annotation(Dialog(enable=zerTim == ProsNet.Utilities.Time.Types.ZeroTime.Custom));
    parameter Boolean outputUnixTimeStamp = false
      "= true, to output the unix time stamp (using GMT reference)"
      annotation(Dialog(group="Unix time stamp"));
    parameter Modelica.Units.SI.Time timZon(displayUnit="h") = 0
      "The local time zone, for computing the unix time stamp only"
      annotation (Dialog(enable=outputUnixTimeStamp, group="Unix time stamp"));
    parameter Modelica.Units.SI.Time offset(displayUnit="h") = 0
      "Offset that is added to 'time', may be used for computing time in different time zones"
      annotation (Dialog(tab="Advanced"));

    Modelica.Blocks.Interfaces.RealOutput unixTimeStampLocal(final unit="s")
      "Unix time stamp at local time"
          annotation (Placement(transformation(extent={{100,-78},{120,-58}}),
          iconTransformation(extent={{100,-78},{120,-58}})));
    Modelica.Blocks.Interfaces.RealOutput unixTimeStamp(final unit="s") = unixTimeStampLocal - timZon if outputUnixTimeStamp
      "Unix time stamp"
          annotation (Placement(transformation(extent={{100,-100},{120,-80}}),
          iconTransformation(extent={{100,-100},{120,-80}})));
    discrete Modelica.Blocks.Interfaces.IntegerOutput year "Year"
      annotation (Placement(transformation(extent={{100,-24},{120,-4}}),
          iconTransformation(extent={{100,-24},{120,-4}})));
    discrete Modelica.Blocks.Interfaces.IntegerOutput month "Month of the year"
      annotation (Placement(transformation(extent={{100,2},{120,22}}),
          iconTransformation(extent={{100,2},{120,22}})));
    Modelica.Blocks.Interfaces.IntegerOutput day(fixed=false) "Day of the month"
      annotation (Placement(transformation(extent={{100,28},{120,48}}),
          iconTransformation(extent={{100,28},{120,48}})));
    Modelica.Blocks.Interfaces.IntegerOutput hour(fixed=false) "Hour of the day"
      annotation (Placement(transformation(extent={{100,54},{120,74}}),
          iconTransformation(extent={{100,54},{120,74}})));
    Modelica.Blocks.Interfaces.RealOutput minute "Minute of the hour"
      annotation (Placement(transformation(extent={{100,80},{120,100}}),
          iconTransformation(extent={{100,80},{120,100}})));
    Modelica.Blocks.Interfaces.IntegerOutput weekDay(fixed=false)
      "Integer output representing week day (monday = 1, sunday = 7)"
      annotation (Placement(transformation(extent={{100,-50},{120,-30}}),
          iconTransformation(extent={{100,-50},{120,-30}})));

  protected
    final constant Real eps_time(final unit="s") = 1 "Small value for time";
    final constant Integer firstYear = 2010
      "First year that is supported, i.e. the first year in timeStampsNewYear[:]";
    final constant Integer lastYear = firstYear + size(timeStampsNewYear,1) - 1;
    constant Modelica.Units.SI.Time timeStampsNewYear[42]={1262304000.0,
      1293840000.0,1325376000.0,1356998400.0,1388534400.0,1420070400.0,
      1451606400.0,1483228800.0,1514764800.0,1546300800.0,1577836800.0,
      1609459200.0,1640995200.0,1672531200.0,1704067200.0,1735689600.0,
      1767225600.0,1798761600.0,1830297600.0,1861920000.0,1893456000.0,
      1924992000.0,1956528000.0,1988150400.0,2019686400.0,2051222400.0,
      2082758400.0,2114380800.0,2145916800.0,2177452800.0,2208988800.0,
      2240611200.0,2272147200.0,2303683200.0,2335219200.0,2366841600.0,
      2398377600.0,2429913600.0,2461449600.0,2493072000.0,2524608000.0,
      2556144000.0} "Epoch time stamps for new years day 2010 to 2051";
    constant Boolean isLeapYear[41] = {
      false, false, true, false,
      false, false, true, false,
      false, false, true, false,
      false, false, true, false,
      false, false, true, false,
      false, false, true, false,
      false, false, true, false,
      false, false, true, false,
      false, false, true, false,
      false, false, true, false,
      false}
      "List of leap years starting from firstYear (2010), up to and including 2050";
    final constant Integer dayInMonth[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31}
      "Number of days in each month";
    parameter Modelica.Units.SI.Time timOff(fixed=false) "Time offset";
    // final parameters since the user may wrongly assume that this model shifts the
    // actual time of the simulation
    final constant Integer monthRef(min=1, max=12) = 1 "Month when time = 0"
      annotation(Dialog(enable=zerTim == ProsNet.Utilities.Time.Types.ZeroTime.Custom));
    final constant Integer dayRef(min=1, max=31) = 1 "Day when time = 0"
      annotation(Dialog(enable=zerTim == ProsNet.Utilities.Time.Types.ZeroTime.Custom));
    Integer daysSinceEpoch(fixed=false) "Number of days that passed since 1st of January 1970";
    discrete Integer yearIndex "Index of the current year in timeStampsNewYear";
    discrete Real epochLastMonth
      "Unix time stamp of the beginning of the current month";

    final parameter Modelica.Units.SI.Time hourSampleStart(fixed=false)
      "Time when the sampling every hour starts";
    final parameter Modelica.Units.SI.Time daySampleStart(fixed=false)
      "Time when the sampling every day starts";

    Boolean hourSampleTrigger "True, if hourly sample time instant";
    Boolean daySampleTrigger "True, if daily sample time instant";

    Boolean firstHourSampling(fixed=true, start=true)
      "=true if the hour is sampled the first time";
    Boolean firstDaySampling(fixed=true, start=true)
      "=true if the day is sampled the first time";
  initial equation
    hourSampleStart = integer(time/3600)*3600 - offset;
    daySampleStart  = integer(time/(3600*24))*3600*24 - offset;

    hour = integer(floor(rem(unixTimeStampLocal,3600*24)/3600));
    daysSinceEpoch = integer(floor(unixTimeStampLocal/3600/24));

    day = integer(1+floor((unixTimeStampLocal-epochLastMonth)/3600/24));
    weekDay = integer(rem(4+daysSinceEpoch-1,7)+1);
  initial algorithm
    // check if yearRef is in the valid range
    assert(not zerTim == ProsNet.Utilities.Time.Types.ZeroTime.Custom or
      yearRef >= firstYear and yearRef <= lastYear,
      "The value you chose for yearRef (=" + String(yearRef) + ") is outside of
   the validity range of " + String(firstYear) + " to " + String(lastYear) +
      ".");

    // check if the day number exists for the chosen month and year
    assert(not zerTim == ProsNet.Utilities.Time.Types.ZeroTime.Custom or
      dayInMonth[monthRef] + (if monthRef == 2 and isLeapYear[yearRef -
      firstYear + 1] then 1 else 0) >= dayRef,
      "The day number you chose is larger than the number of days contained by the month you chose.");

    // compute the offset to be added to time based on the parameters specified by the user
    if zerTim == ProsNet.Utilities.Time.Types.ZeroTime.UnixTimeStamp then
      timOff :=0;
    elseif zerTim == ProsNet.Utilities.Time.Types.ZeroTime.UnixTimeStampGMT then
      timOff :=timZon;
    elseif zerTim == ProsNet.Utilities.Time.Types.ZeroTime.NY2010 or zerTim ==
        ProsNet.Utilities.Time.Types.ZeroTime.Custom and yearRef == 2010 then
        timOff :=timeStampsNewYear[1];
    elseif zerTim == ProsNet.Utilities.Time.Types.ZeroTime.NY2011 or zerTim ==
        ProsNet.Utilities.Time.Types.ZeroTime.Custom and yearRef == 2011 then
        timOff :=timeStampsNewYear[2];
    elseif zerTim == ProsNet.Utilities.Time.Types.ZeroTime.NY2012 or zerTim ==
        ProsNet.Utilities.Time.Types.ZeroTime.Custom and yearRef == 2012 then
        timOff :=timeStampsNewYear[3];
    elseif zerTim == ProsNet.Utilities.Time.Types.ZeroTime.NY2013 or zerTim ==
        ProsNet.Utilities.Time.Types.ZeroTime.Custom and yearRef == 2013 then
        timOff :=timeStampsNewYear[4];
    elseif zerTim == ProsNet.Utilities.Time.Types.ZeroTime.NY2014 or zerTim ==
        ProsNet.Utilities.Time.Types.ZeroTime.Custom and yearRef == 2014 then
        timOff :=timeStampsNewYear[5];
    elseif zerTim == ProsNet.Utilities.Time.Types.ZeroTime.NY2015 or zerTim ==
        ProsNet.Utilities.Time.Types.ZeroTime.Custom and yearRef == 2015 then
        timOff :=timeStampsNewYear[6];
    elseif zerTim == ProsNet.Utilities.Time.Types.ZeroTime.NY2016 or zerTim ==
        ProsNet.Utilities.Time.Types.ZeroTime.Custom and yearRef == 2016 then
        timOff :=timeStampsNewYear[7];
    elseif zerTim == ProsNet.Utilities.Time.Types.ZeroTime.NY2017 or zerTim ==
        ProsNet.Utilities.Time.Types.ZeroTime.Custom and yearRef == 2017 then
        timOff :=timeStampsNewYear[8];
    elseif zerTim == ProsNet.Utilities.Time.Types.ZeroTime.NY2018 or zerTim ==
        ProsNet.Utilities.Time.Types.ZeroTime.Custom and yearRef == 2018 then
        timOff :=timeStampsNewYear[9];
    elseif zerTim == ProsNet.Utilities.Time.Types.ZeroTime.NY2019 or zerTim ==
        ProsNet.Utilities.Time.Types.ZeroTime.Custom and yearRef == 2019 then
        timOff :=timeStampsNewYear[10];
    elseif zerTim == ProsNet.Utilities.Time.Types.ZeroTime.NY2020 or zerTim ==
        ProsNet.Utilities.Time.Types.ZeroTime.Custom and yearRef == 2020 then
        timOff :=timeStampsNewYear[11];
    elseif zerTim == ProsNet.Utilities.Time.Types.ZeroTime.NY2021 or zerTim ==
        ProsNet.Utilities.Time.Types.ZeroTime.Custom and yearRef == 2021 then
        timOff :=timeStampsNewYear[12];
    elseif zerTim == ProsNet.Utilities.Time.Types.ZeroTime.NY2022 or zerTim ==
        ProsNet.Utilities.Time.Types.ZeroTime.Custom and yearRef == 2022 then
        timOff :=timeStampsNewYear[13];
    elseif zerTim == ProsNet.Utilities.Time.Types.ZeroTime.NY2023 or zerTim ==
        ProsNet.Utilities.Time.Types.ZeroTime.Custom and yearRef == 2023 then
        timOff :=timeStampsNewYear[14];
    elseif zerTim == ProsNet.Utilities.Time.Types.ZeroTime.NY2024 or zerTim ==
        ProsNet.Utilities.Time.Types.ZeroTime.Custom and yearRef == 2024 then
        timOff := timeStampsNewYear[14];
    elseif zerTim == ProsNet.Utilities.Time.Types.ZeroTime.NY2025 or zerTim ==
        ProsNet.Utilities.Time.Types.ZeroTime.Custom and yearRef == 2025 then
        timOff := timeStampsNewYear[15];
    elseif zerTim == ProsNet.Utilities.Time.Types.ZeroTime.NY2026 or zerTim ==
        ProsNet.Utilities.Time.Types.ZeroTime.Custom and yearRef == 2026 then
        timOff := timeStampsNewYear[16];
    elseif zerTim == ProsNet.Utilities.Time.Types.ZeroTime.NY2027 or zerTim ==
        ProsNet.Utilities.Time.Types.ZeroTime.Custom and yearRef == 2027 then
        timOff := timeStampsNewYear[17];
    elseif zerTim == ProsNet.Utilities.Time.Types.ZeroTime.NY2028 or zerTim ==
        ProsNet.Utilities.Time.Types.ZeroTime.Custom and yearRef == 2028 then
        timOff := timeStampsNewYear[18];
    elseif zerTim == ProsNet.Utilities.Time.Types.ZeroTime.NY2029 or zerTim ==
        ProsNet.Utilities.Time.Types.ZeroTime.Custom and yearRef == 2029 then
        timOff := timeStampsNewYear[19];
    elseif zerTim == ProsNet.Utilities.Time.Types.ZeroTime.NY2030 or zerTim ==
        ProsNet.Utilities.Time.Types.ZeroTime.Custom and yearRef == 2030 then
        timOff := timeStampsNewYear[20];
    elseif zerTim == ProsNet.Utilities.Time.Types.ZeroTime.NY2031 or zerTim ==
        ProsNet.Utilities.Time.Types.ZeroTime.Custom and yearRef == 2031 then
        timOff := timeStampsNewYear[21];
    elseif zerTim == ProsNet.Utilities.Time.Types.ZeroTime.NY2032 or zerTim ==
        ProsNet.Utilities.Time.Types.ZeroTime.Custom and yearRef == 2032 then
        timOff := timeStampsNewYear[22];
    elseif zerTim == ProsNet.Utilities.Time.Types.ZeroTime.NY2033 or zerTim ==
        ProsNet.Utilities.Time.Types.ZeroTime.Custom and yearRef == 2033 then
        timOff := timeStampsNewYear[23];
    elseif zerTim == ProsNet.Utilities.Time.Types.ZeroTime.NY2034 or zerTim ==
        ProsNet.Utilities.Time.Types.ZeroTime.Custom and yearRef == 2034 then
        timOff := timeStampsNewYear[24];
    elseif zerTim == ProsNet.Utilities.Time.Types.ZeroTime.NY2035 or zerTim ==
        ProsNet.Utilities.Time.Types.ZeroTime.Custom and yearRef == 2035 then
        timOff := timeStampsNewYear[25];
    elseif zerTim == ProsNet.Utilities.Time.Types.ZeroTime.NY2036 or zerTim ==
        ProsNet.Utilities.Time.Types.ZeroTime.Custom and yearRef == 2036 then
        timOff := timeStampsNewYear[26];
    elseif zerTim == ProsNet.Utilities.Time.Types.ZeroTime.NY2037 or zerTim ==
        ProsNet.Utilities.Time.Types.ZeroTime.Custom and yearRef == 2037 then
        timOff := timeStampsNewYear[27];
    elseif zerTim == ProsNet.Utilities.Time.Types.ZeroTime.NY2038 or zerTim ==
        ProsNet.Utilities.Time.Types.ZeroTime.Custom and yearRef == 2038 then
        timOff := timeStampsNewYear[28];
    elseif zerTim == ProsNet.Utilities.Time.Types.ZeroTime.NY2039 or zerTim ==
        ProsNet.Utilities.Time.Types.ZeroTime.Custom and yearRef == 2039 then
        timOff := timeStampsNewYear[29];
    elseif zerTim == ProsNet.Utilities.Time.Types.ZeroTime.NY2040 or zerTim ==
        ProsNet.Utilities.Time.Types.ZeroTime.Custom and yearRef == 2040 then
        timOff := timeStampsNewYear[30];
    elseif zerTim == ProsNet.Utilities.Time.Types.ZeroTime.NY2041 or zerTim ==
        ProsNet.Utilities.Time.Types.ZeroTime.Custom and yearRef == 2041 then
        timOff := timeStampsNewYear[31];
    elseif zerTim == ProsNet.Utilities.Time.Types.ZeroTime.NY2042 or zerTim ==
        ProsNet.Utilities.Time.Types.ZeroTime.Custom and yearRef == 2042 then
        timOff := timeStampsNewYear[32];
    elseif zerTim == ProsNet.Utilities.Time.Types.ZeroTime.NY2043 or zerTim ==
        ProsNet.Utilities.Time.Types.ZeroTime.Custom and yearRef == 2043 then
        timOff := timeStampsNewYear[33];
    elseif zerTim == ProsNet.Utilities.Time.Types.ZeroTime.NY2044 or zerTim ==
        ProsNet.Utilities.Time.Types.ZeroTime.Custom and yearRef == 2044 then
        timOff := timeStampsNewYear[34];
    elseif zerTim == ProsNet.Utilities.Time.Types.ZeroTime.NY2045 or zerTim ==
        ProsNet.Utilities.Time.Types.ZeroTime.Custom and yearRef == 2045 then
        timOff := timeStampsNewYear[35];
    elseif zerTim == ProsNet.Utilities.Time.Types.ZeroTime.NY2046 or zerTim ==
        ProsNet.Utilities.Time.Types.ZeroTime.Custom and yearRef == 2046 then
        timOff := timeStampsNewYear[36];
    elseif zerTim == ProsNet.Utilities.Time.Types.ZeroTime.NY2047 or zerTim ==
        ProsNet.Utilities.Time.Types.ZeroTime.Custom and yearRef == 2047 then
        timOff := timeStampsNewYear[37];
    elseif zerTim == ProsNet.Utilities.Time.Types.ZeroTime.NY2048 or zerTim ==
        ProsNet.Utilities.Time.Types.ZeroTime.Custom and yearRef == 2048 then
        timOff := timeStampsNewYear[38];
    elseif zerTim == ProsNet.Utilities.Time.Types.ZeroTime.NY2049 or zerTim ==
        ProsNet.Utilities.Time.Types.ZeroTime.Custom and yearRef == 2049 then
        timOff := timeStampsNewYear[39];
    elseif zerTim == ProsNet.Utilities.Time.Types.ZeroTime.NY2050 or zerTim ==
        ProsNet.Utilities.Time.Types.ZeroTime.Custom and yearRef == 2050 then
        timOff := timeStampsNewYear[40];
    else
      timOff :=0;
      // this code should not be reachable
      assert(false, "No valid ZeroTime could be identified.
   This is a bug, please submit a bug report.");
    end if;

    // add additional offset when using a custom date and time
    if zerTim == ProsNet.Utilities.Time.Types.ZeroTime.Custom then
      timOff :=timOff + ((dayRef - 1) + sum({dayInMonth[i] for i in 1:(monthRef - 1)})
       + (if monthRef > 2 and isLeapYear[yearRef - firstYear + 1] then 1 else 0))*3600*24;
    end if;

     // input data range checks at initial time
    assert(time + offset + timOff >= timeStampsNewYear[1], if zerTim == ProsNet.Utilities.Time.Types.ZeroTime.UnixTimeStamp
       then "Could not initialize date in the CalendarTime block.
   You selected 1970 as the time=0 reference.
   Therefore the simulation startTime must be at least " + String(
      timeStampsNewYear[1]) + "." elseif zerTim == ProsNet.Utilities.Time.Types.ZeroTime.Custom
       then if yearRef < firstYear then "Could not initialize date in the CalendarTime block.
   You selected a custom time=0 reference.
   The minimum value for yearRef is then " + String(firstYear) +
      " but your value is " + String(yearRef) + "." else "Could not initialize date in the CalendarTime block.
   You selected a custom time=0 reference.
   Possibly your startTime is too small." else "Could not initialize date in the CalendarTime block.
   Possibly your startTime is negative?");

    assert(time + offset + timOff < timeStampsNewYear[size(timeStampsNewYear, 1)],
      if zerTim == ProsNet.Utilities.Time.Types.ZeroTime.Custom and yearRef >=
      lastYear then "Could not initialize date in the CalendarTime block.
   You selected a custom time=0 reference.
   The maximum value for yearRef is then " + String(lastYear) +
      " but your value is " + String(yearRef) + "." else "Could not initialize date in the CalendarTime block.
       Possibly your startTime is too large.");

    // iterate to find the year at initialization
  initial algorithm
    year :=0;
    for i in 1:size(timeStampsNewYear,1) loop
      // may be reformulated using break if JModelica fixes bug
      if unixTimeStampLocal < timeStampsNewYear[i]
        and (if i == 1 then true else unixTimeStampLocal >= timeStampsNewYear[i-1]) then
        yearIndex :=i - 1;
        year :=firstYear + i - 2;
      end if;
    end for;

    // iterate to find the month at initialization
    epochLastMonth := timeStampsNewYear[yearIndex];
    month:=13;
    for i in 1:12 loop
      if (unixTimeStampLocal-epochLastMonth)/3600/24 <
        (if i==2 and isLeapYear[yearIndex] then 1 + dayInMonth[i] else dayInMonth[i]) then
        // construction below avoids the need of a break, which bugs out JModelica
        month :=min(i,month);
      else
        epochLastMonth :=epochLastMonth + (if i == 2 and isLeapYear[yearIndex]
           then 1 + dayInMonth[i] else dayInMonth[i])*3600*24;
      end if;
    end for;

  equation
    // compute unix time step based on found offset
    unixTimeStampLocal = time + offset + timOff;

    // compute other variables that can be computed without using when() statements
    hourSampleTrigger =sample(hourSampleStart, 3600);
    when hourSampleTrigger then
      if pre(firstHourSampling) then
        hour = integer(floor(rem(unixTimeStampLocal,3600*24)/3600));
      else
        hour = if (pre(hour) == 23) then 0 else (pre(hour) + 1);
      end if;
      firstHourSampling = false;
    end when;

    daySampleTrigger =sample(daySampleStart, 86400);
    when daySampleTrigger then
      if pre(firstDaySampling) then
        daysSinceEpoch = integer(floor(unixTimeStampLocal/3600/24));
        weekDay=integer(rem(4+daysSinceEpoch-1,7)+1);

      else
        daysSinceEpoch = pre(daysSinceEpoch) + 1;
        weekDay = if (pre(weekDay) == 7) then 1 else (pre(weekDay) + 1);
      end if;

      // update the year when passing the epoch time stamp of the next year
      if unixTimeStampLocal - timeStampsNewYear[pre(yearIndex)+1] > -eps_time then
        yearIndex=pre(yearIndex)+1;
        year = pre(year) + 1;
      else
        yearIndex = pre(yearIndex);
        year = pre(year);
      end if;
      assert(yearIndex<=size(timeStampsNewYear,1),
        "Index out of range for epoch vector: timeStampsNewYear needs to be extended beyond the year "
          + String(firstYear+size(timeStampsNewYear,1)));

      // update the month when passing the last day of the current month
      if unixTimeStampLocal - ( pre(epochLastMonth) + (if pre(month)==2 and isLeapYear[yearIndex] then 1 + dayInMonth[pre(month)] else dayInMonth[pre(month)])*3600*24)  > -eps_time then
        month = if pre(month) == 12 then 1 else pre(month) + 1;
        // Use floor(0.1 + ...) to avoid floating point errors when accumulating epochLastMonth.
        epochLastMonth = floor(0.1 + pre(epochLastMonth) +
          (if pre(month)==2 and isLeapYear[yearIndex]
            then 1 + dayInMonth[pre(month)] else dayInMonth[pre(month)])*3600*24);
      else
        month = pre(month);
        epochLastMonth = pre(epochLastMonth);
      end if;

      day = integer(1+floor((unixTimeStampLocal-epochLastMonth)/3600/24));

      firstDaySampling = false;
    end when;

    // using Real variables and operations for minutes since otherwise too many events are generated
    minute = (unixTimeStampLocal/60-daysSinceEpoch*60*24-hour*60);

    annotation (
      defaultComponentName="calTim",
    Documentation(revisions="<html>
<ul>
<li>
March 8, 2024, by Jelger Jansen:<br/>
Allow reference years later than 2020 and extend functionality to year 2050.<br/>
This is for 
<a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/1847\">#1847</a>.
</li>  
<li>
December 19, 2022, by Michael Wetter:<br/>
Refactored implementation to avoid wrong day number due to rounding errors that caused simultaneous events
to not be triggered at the same time.<br/>
This is for
<a href=\"https://github.com/lbl-srg/modelica-buildings/issues/3199\">Buildings, #3199</a>.
</li>
<li>
November 6, 2019, by Milica Grahovac:<br/>
Extended functionality to year 2030.
</li>
<li>
August 20, 2019, by Filip Jorissen:<br/>
Revised implementation such that the meaning of <code>time</code> is better explained
and unix time stamps are correctly defined with respect to GMT.
(see <a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/1192\">#1192</a>).
</li>
<li>
February 14, 2019, by Damien Picard:<br/>
Fix bug when non-zero offset by substracting the offset from hourSampleStart and daySampleStart
(see <a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/1099\">#1099</a>).
</li>
<li>
August 3, 2016, by Filip Jorissen:<br/>
First implementation.
</li>
</ul>
</html>",   info="<html>
<p>
This blocks computes the unix time stamp, date and time
and the day of the week based on the Modelica
variable <code>time</code>.
As for the weather data reader
<a href=\"modelica://Buildings.BoundaryConditions.WeatherData.ReaderTMY3\">
Buildings.BoundaryConditions.WeatherData.ReaderTMY3</a>,
<code>time=0</code> corresponds to January 1st at midnight
in the <em>local time zone</em>.
The computed outputs are thus also for the local time zone.
The year for which <code>time=0</code> is determined by
the parameter <code>zerTim</code>.
</p>
<h4>Main equations</h4>
<p>
The unix time stamp corresponding to the current time is computed.
From this variables the corresponding, year, date and time are computed using functions
such as <code>floor()</code> and <code>ceil()</code>.
</p>
<h4>Assumption and limitations</h4>
<p>
The implementation only supports date computations from year 2010 up to and including 2020.
Daylight saving is not supported.
</p>
<h4>Typical use and important parameters</h4>
<p>
The user must define which time and date correspond to <code>time = 0</code>
using the model parameters <code>zerTim</code>, and, if
<code>zerTim==Buildings.Utilities.Time.Types.ZeroTime.Custom</code>,
the parameter <code>yearRef</code>.
When <code>zerTim==Buildings.Utilities.Time.Types.ZeroTime.UnixTimeStampGMT</code>,
<code>time</code> is defined with respect to GMT. This is different from the use
of <code>time</code> in the weather data reader
<a href=\"modelica://Buildings.BoundaryConditions.WeatherData.ReaderTMY3\">
Buildings.BoundaryConditions.WeatherData.ReaderTMY3</a>, as the weather data files
used with this reader are generally defined with <code>time</code> being local time.
If  <code>zerTim==Buildings.Utilities.Time.Types.ZeroTime.UnixTimeStampGMT</code> is used,
then the weather data files read by
<a href=\"modelica://Buildings.BoundaryConditions.WeatherData.ReaderTMY3\">
Buildings.BoundaryConditions.WeatherData.ReaderTMY3</a>
must also be defined with GMT as the time stamp.

The user can choose from new year, midnight for a number of years:
2010 to 2050 and also 1970.
The latter corresponds to a unix stamp of <i>0</i>.
(Note that when choosing the reference time equal to 0 at 1970,
the actual simulation time must be within the 2010-2051 range.
For instance <code>startTime = 1262304000</code> corresponds
to the simulation starting on the 1st of January 2010
when setting <code>zerTim = ZeroTime.UnixTimeStamp</code>.
This is within the 2010-2050 range and is therefore allowed.)
The unix time stamp is formally defined as the number of
seconds since midnight of new year in 1970 GMT.
To output the correct unix time stamp, set <code>outputUnixTimeStamp=true</code>
We then require the local time zone <code>timZon</code>
(see <a href=\"modelica://Buildings.BoundaryConditions.WeatherData.ReaderTMY3\">
Buildings.BoundaryConditions.WeatherData.ReaderTMY3</a>)
since <code>time</code> uses the local time zone instead of GMT.
We always output <code>unixTimeStampLocal</code>, which is a time stamp
that uses the local time zone reference instead of GMT.
</p>
<h4>Implementation</h4>
<p>
The model was implemented such that no events are being generated for computing the minute of the day.
The model also contains an implementation for setting <code>time=0</code>
for any day and month other than January first.
This is however not activated in the current model since these options may wrongly give the impression
that it changes the time based on which the solar position is computed and TMY3 data are read.
</p>
</html>"),
      Icon(graphics={
          Text(
            extent={{-34,90},{96,80}},
            textColor={28,108,200},
            horizontalAlignment=TextAlignment.Right,
            textString="Minute"),
          Text(
            extent={{-28,68},{96,58}},
            textColor={28,108,200},
            horizontalAlignment=TextAlignment.Right,
            textString="Hour"),
          Text(
            extent={{-38,44},{96,32}},
            textColor={28,108,200},
            horizontalAlignment=TextAlignment.Right,
            textString="Day"),
          Text(
            extent={{-50,18},{96,8}},
            textColor={28,108,200},
            horizontalAlignment=TextAlignment.Right,
            textString="Month"),
          Text(
            extent={{-70,-8},{96,-18}},
            textColor={28,108,200},
            horizontalAlignment=TextAlignment.Right,
            textString="Year"),
          Text(
            extent={{-68,-30},{96,-42}},
            textColor={28,108,200},
            horizontalAlignment=TextAlignment.Right,
            textString="Weekday"),
          Text(
            extent={{-102,-60},{94,-72}},
            textColor={28,108,200},
            horizontalAlignment=TextAlignment.Right,
            textString="Unix time stamp (local)"),
          Ellipse(
            extent={{-94,94},{16,-16}},
            lineColor={160,160,164},
            fillColor={215,215,215},
            fillPattern=FillPattern.Solid),
          Line(
            points={{-40,38},{-64,62}},
            thickness=0.5),
          Line(
            points={{-40,38},{-14,38}},
            thickness=0.5),
          Text(
            extent={{-102,-82},{94,-94}},
            textColor={28,108,200},
            horizontalAlignment=TextAlignment.Right,
            visible=outputUnixTimeStamp,
            textString="Unix time stamp (GMT)")}));
  end CalendarTime;

  block ModelTime "Model time"
    extends Modelica.Blocks.Icons.Block;

    Modelica.Blocks.Interfaces.RealOutput y "Model time"
      annotation (Placement(transformation(extent={{100,-10},{120,10}})));

  equation
    y = time;

    annotation (
      defaultComponentName="modTim",
      Icon(coordinateSystem(
          preserveAspectRatio=true,
          extent={{-100,-100},{100,100}},
          grid={1,1}), graphics={
          Ellipse(extent={{-80,80},{80,-80}}, lineColor={160,160,164},
            fillColor={215,215,215},
            fillPattern=FillPattern.Solid),
          Line(points={{0,80},{0,60}}, color={160,160,164}),
          Line(points={{80,0},{60,0}}, color={160,160,164}),
          Line(points={{0,-80},{0,-60}}, color={160,160,164}),
          Line(points={{-80,0},{-60,0}}, color={160,160,164}),
          Line(points={{37,70},{26,50}}, color={160,160,164}),
          Line(points={{70,38},{49,26}}, color={160,160,164}),
          Line(points={{71,-37},{52,-27}}, color={160,160,164}),
          Line(points={{39,-70},{29,-51}}, color={160,160,164}),
          Line(points={{-39,-70},{-29,-52}}, color={160,160,164}),
          Line(points={{-71,-37},{-50,-26}}, color={160,160,164}),
          Line(points={{-71,37},{-54,28}}, color={160,160,164}),
          Line(points={{-38,70},{-28,51}}, color={160,160,164}),
          Line(
            points={{0,0},{-50,50}},
            thickness=0.5),
          Line(
            points={{0,0},{40,0}},
            thickness=0.5)}),
      Documentation(info="<html>
<p>
This component outputs the model time, which starts at the value at which the simulation starts.
For example, if a simulation starts at <i>t=-1</i>, then this block outputs first <i>t=-1</i>,
and its output is advanced at the same rate as the simulation time.
</p>
<p>
The model is used to allow the simulation to start from any time without having to set
the parameters for the clock, as would be necessary for the model
<a href=\"modelica://Modelica.Blocks.Sources.ContinuousClock\">Modelica.Blocks.Sources.ContinuousClock</a>.
</p>
</html>",   revisions="<html>
<ul>
<li>
May 2, 2022, by Michael Wetter:<br/>
Corrected hyperlink in documentation.<br/>
This is for
<a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/1609\">IBPSA, #1609</a>.
</li>
<li>
April 17, 2020, by Michael Wetter:<br/>
Refactored so that the output connector has a better comment string, because
this comment string is displayed on the weather data bus.
</li>
<li>
January 16, 2015, by Michael Wetter:<br/>
Moved block from
<code>Buildings.Utilities.SimulationTime</code>
to
<code>Buildings.Utilities.Time.ModelTime</code>.
</li>
<li>
May 18, 2010, by Wangda Zuo:<br/>
First implementation.
</li>
</ul>
</html>"));
  end ModelTime;

  package Types "Package with type definitions"
   extends Modelica.Icons.TypesPackage;

    type ZeroTime = enumeration(
        UnixTimeStamp "Thu, 01 Jan 1970 00:00:00 local time",
        UnixTimeStampGMT "Thu, 01 Jan 1970 00:00:00 GMT",
        Custom "User specified local time",
        NY2010 "New year 2010, 00:00:00 local time",
        NY2011 "New year 2011, 00:00:00 local time",
        NY2012 "New year 2012, 00:00:00 local time",
        NY2013 "New year 2013, 00:00:00 local time",
        NY2014 "New year 2014, 00:00:00 local time",
        NY2015 "New year 2015, 00:00:00 local time",
        NY2016 "New year 2016, 00:00:00 local time",
        NY2017 "New year 2017, 00:00:00 local time",
        NY2018 "New year 2018, 00:00:00 local time",
        NY2019 "New year 2019, 00:00:00 local time",
        NY2020 "New year 2020, 00:00:00 local time",
        NY2021 "New year 2021, 00:00:00 local time",
        NY2022 "New year 2022, 00:00:00 local time",
        NY2023 "New year 2023, 00:00:00 local time",
        NY2024 "New year 2024, 00:00:00 local time",
        NY2025 "New year 2025, 00:00:00 local time",
        NY2026 "New year 2026, 00:00:00 local time",
        NY2027 "New year 2027, 00:00:00 local time",
        NY2028 "New year 2028, 00:00:00 local time",
        NY2029 "New year 2029, 00:00:00 local time",
        NY2030 "New year 2030, 00:00:00 local time",
        NY2031 "New year 2031, 00:00:00 local time",
        NY2032 "New year 2032, 00:00:00 local time",
        NY2033 "New year 2033, 00:00:00 local time",
        NY2034 "New year 2034, 00:00:00 local time",
        NY2035 "New year 2035, 00:00:00 local time",
        NY2036 "New year 2036, 00:00:00 local time",
        NY2037 "New year 2037, 00:00:00 local time",
        NY2038 "New year 2038, 00:00:00 local time",
        NY2039 "New year 2039, 00:00:00 local time",
        NY2040 "New year 2040, 00:00:00 local time",
        NY2041 "New year 2041, 00:00:00 local time",
        NY2042 "New year 2042, 00:00:00 local time",
        NY2043 "New year 2043, 00:00:00 local time",
        NY2044 "New year 2044, 00:00:00 local time",
        NY2045 "New year 2045, 00:00:00 local time",
        NY2046 "New year 2046, 00:00:00 local time",
        NY2047 "New year 2047, 00:00:00 local time",
        NY2048 "New year 2048, 00:00:00 local time",
        NY2049 "New year 2049, 00:00:00 local time",
        NY2050 "New year 2050, 00:00:00 local time")
      "Use this to set the date corresponding to time = 0"
      annotation (Documentation(info="<html>
<p>
Type for choosing how to set the reference time in
<a href=\"modelica://Buildings.Utilities.Time.CalendarTime\">
Buildings.Utilities.Time.CalendarTime</a>.
</p>
<p>
For example, <code>Buildings.Utilities.Time.Types.TimeReference.NY2016</code>
means that if the Modelica built-in variable <code>time=0</code>, it is
January 1, 2016, 0:00:00 local time.
</p>
<p>
When using <code>Buildings.Utilities.Time.Types.ZeroTime.UnixTimeStampGMT</code>,
<code>time</code> is defined with respect to GMT. This is different from the use
of <code>time</code> in the weather data reader
<a href=\"modelica://Buildings.BoundaryConditions.WeatherData.ReaderTMY3\">
Buildings.BoundaryConditions.WeatherData.ReaderTMY3</a>, as the weather data reader assumes
that <code>time</code> is expressed in local time.
</p>
</html>",     revisions="<html>
<ul>
<li>
March 8, 2024, by Jelger Jansen:<br/>
Extend functionality to year 2050.<br/>
This is for 
<a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/1847\">#1847</a>.
</li> 
<li>
September 10, 2016, by Michael Wetter:<br/>
Revised implementation and moved to new package
<a href=\"modelica://Buildings.Utilities.Time.CalendarTime.Types\">
Buildings.Utilities.Time.CalendarTime.Types</a>.
</li>
<li>
August 3, 2016, by Filip Jorissen:<br/>
First implementation.
</li>
</ul>
</html>"));
  annotation (preferredView="info", Documentation(info="<html>
This package contains type definitions.
</html>"));
  end Types;

  package Examples "Collection of models that illustrate model use and test models"
    extends Modelica.Icons.ExamplesPackage;

    model CalendarTime "Example for the calendar time model"
      extends Modelica.Icons.Example;
      ProsNet.Utilities.Time.CalendarTime calendarTime2016(zerTim=ProsNet.Utilities.Time.Types.ZeroTime.NY2016)
        "Computes date and time assuming time=0 corresponds to new year 2016"
        annotation (Placement(transformation(extent={{-8,-10},{12,10}})));

    equation

      annotation (    __Dymola_Commands(file="modelica://Buildings/Resources/Scripts/Dymola/Utilities/Time/Examples/CalendarTime.mos"
            "Simulate and plot"),
        Documentation(revisions="<html>
    <ul>
<li>
August 3, 2016, by Filip Jorissen:<br/>
First implementation.
</li>
</ul>
</html>",     info="<html>
<p>
This model demonstrates the use of the
<a href=\"modelica://Buildings.Utilities.Time.CalendarTime\">
Buildings.Utilities.Time.CalendarTime</a>
block.
</p>
</html>"),
        experiment(Tolerance=1e-6, StopTime=1e+08));
    end CalendarTime;

    model ModelTime "Test model for the ModelTime block"
      extends Modelica.Icons.Example;
      ProsNet.Utilities.Time.ModelTime modTim "Model time"
        annotation (Placement(transformation(extent={{-12,-10},{8,10}})));
    equation

      annotation (
      Documentation(info="<html>
<p>
This model tests the implementation of
the block that outputs the model time.
</p>
</html>",     revisions="<html>
<ul>
<li>
January 16, 2015, by Michael Wetter:<br/>
First implementation.
</li>
</ul>
</html>"),
      experiment(
      StartTime=-1,
      Tolerance=1e-6, StopTime=1),
    __Dymola_Commands(file="modelica://Buildings/Resources/Scripts/Dymola/Utilities/Time/Examples/ModelTime.mos"
            "Simulate and plot"));
    end ModelTime;
  annotation (preferredView="info", Documentation(info="<html>
<p>
This package contains examples for the use of models that can be found in
<a href=\"modelica://Buildings.Utilities.Time\">
Buildings.Utilities.Time</a>.
</p>
</html>"));
  end Examples;

  package Validation "Collection of models that validate the time models"
    extends Modelica.Icons.ExamplesPackage;

    model CalendarTimeMonths "Validation model for the calendar time model"
      extends Modelica.Icons.Example;
      ProsNet.Utilities.Time.CalendarTime calTim(zerTim=ProsNet.Utilities.Time.Types.ZeroTime.NY2015)
        "Computes date and time assuming time=0 corresponds to new year 2015"
        annotation (Placement(transformation(extent={{-20,-10},{0,10}})));

    equation

      annotation (
        __Dymola_Commands(file="modelica://Buildings/Resources/Scripts/Dymola/Utilities/Time/Validation/CalendarTimeMonths.mos"
            "Simulate and plot"),
      Documentation(
        info="<html>
<p>
This model validates the use of the
<a href=\"modelica://Buildings.Utilities.Time.CalendarTime\">
Buildings.Utilities.Time.CalendarTime</a>
block for a period of a couple of months.
This shorter simulation time has been selected to
store the reference results that are used in the regression tests
at a resulation that makes sense for the minute and hour outputs.
</p>
</html>",
    revisions="<html>
<ul>
<li>
September 9, 2016, by Michael Wetter:<br/>
First implementation.
</li>
</ul>
</html>"),
        experiment(StartTime=172800, Tolerance=1e-6, StopTime=345600));
    end CalendarTimeMonths;

    model CalendarTimeMonthsMinus
      "Validation model for the calendar time model with start time slightly below the full hour"
      extends ProsNet.Utilities.Time.Validation.CalendarTimeMonths;

      annotation (
        __Dymola_Commands(file="modelica://Buildings/Resources/Scripts/Dymola/Utilities/Time/Validation/CalendarTimeMonthsMinus.mos"
            "Simulate and plot"),
      Documentation(
        info="<html>
<p>
This model validates the use of the
<a href=\"modelica://Buildings.Utilities.Time.CalendarTime\">
Buildings.Utilities.Time.CalendarTime</a>.
It is identical to
<a href=\"modelica://Buildings.Utilities.Time.Validation.CalendarTimeMonths\">
Buildings.Utilities.Time.Validation.CalendarTimeMonths</a>
except that the start and end time are different.
</p>
</html>",
    revisions="<html>
<ul>
<li>
September 14, 2016, by Michael Wetter:<br/>
First implementation.
</li>
</ul>
</html>"),
        experiment(StartTime=172799, Tolerance=1e-6, StopTime=345599));
    end CalendarTimeMonthsMinus;

    model CalendarTimeMonthsPlus
      "Validation model for the calendar time model with start time slightly higher than the full hour"
      extends ProsNet.Utilities.Time.Validation.CalendarTimeMonths;

      annotation (
        __Dymola_Commands(file="modelica://Buildings/Resources/Scripts/Dymola/Utilities/Time/Validation/CalendarTimeMonthsPlus.mos"
            "Simulate and plot"),
      Documentation(
        info="<html>
<p>
This model validates the use of the
<a href=\"modelica://Buildings.Utilities.Time.CalendarTime\">
Buildings.Utilities.Time.CalendarTime</a>.
It is identical to
<a href=\"modelica://Buildings.Utilities.Time.Validation.CalendarTimeMonths\">
Buildings.Utilities.Time.Validation.CalendarTimeMonths</a>
except that the start and end time are different.
</p>
</html>",
    revisions="<html>
<ul>
<li>
September 14, 2016, by Michael Wetter:<br/>
First implementation.
</li>
</ul>
</html>"),
        experiment(StartTime=172801, Tolerance=1e-6, StopTime=345601));
    end CalendarTimeMonthsPlus;
  annotation (preferredView="info", Documentation(info="<html>
<p>
This package contains models that validate the time models.
The examples plot various outputs. These model outputs are stored as reference data to
allow continuous validation whenever models in the library change.
</p>
</html>"));
  end Validation;
annotation (preferredView="info", Documentation(info="<html>
<p>
This package contains models for time.
</p>
</html>"),
  Icon(graphics={
        Ellipse(extent={{-80,80},{80,-80}}, lineColor={160,160,164},
          fillColor={215,215,215},
          fillPattern=FillPattern.Solid),
        Line(points={{0,80},{0,60}}, color={160,160,164}),
        Line(points={{80,0},{60,0}}, color={160,160,164}),
        Line(points={{0,-80},{0,-60}}, color={160,160,164}),
        Line(points={{-80,0},{-60,0}}, color={160,160,164}),
        Line(points={{37,70},{26,50}}, color={160,160,164}),
        Line(points={{70,38},{49,26}}, color={160,160,164}),
        Line(points={{71,-37},{52,-27}}, color={160,160,164}),
        Line(points={{39,-70},{29,-51}}, color={160,160,164}),
        Line(points={{-39,-70},{-29,-52}}, color={160,160,164}),
        Line(points={{-71,-37},{-50,-26}}, color={160,160,164}),
        Line(points={{-71,37},{-54,28}}, color={160,160,164}),
        Line(points={{-38,70},{-28,51}}, color={160,160,164}),
        Line(
          points={{0,0},{-50,50}},
          thickness=0.5),
        Line(
          points={{0,0},{40,0}},
          thickness=0.5)}));
end Time;
