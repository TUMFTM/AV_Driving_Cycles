function [ TumColors ] = tumColors(  )
%tumColors creates struct with TUM corporate identity colors for use in
%plots etc.
%   S. Wolff 29.06.2018

TumColors.primary.Blue =         [0 101 189] ./255;
TumColors.primary.Black =        [0 0 0] ./255;
TumColors.primary.White =        [255 255 255] ./255;

TumColors.secondary.LightBlue =  [0 82 147] ./255;
TumColors.secondary.DarkBlue =   [0 51 89] ./255;
TumColors.secondary.LightGrey =  [217 218 219] ./255;
TumColors.secondary.Grey =       [156 157 159] ./255;
TumColors.secondary.DarkGrey =   [88 88 90] ./255;

TumColors.accent.Orange =        [227 114 34] ./255;
TumColors.accent.Green =         [162 173 0] ./255;
TumColors.accent.LightBlue =     [152 198 234] ./255;
TumColors.accent.DarkBlue =      [100 160 200] ./255;
TumColors.accent.Beige =         [218 215 203] ./255;

TumColors.Extended.E1 =           [105  8  90] ./255;
TumColors.Extended.E2 =           [15  27  9] ./255;
TumColors.Extended.E3 =           [50  119  138] ./255;
TumColors.Extended.E4 =           [0  124  48] ./255;
TumColors.Extended.E5 =           [103  154  29] ./255;
TumColors.Extended.E6 =           [255  220  0] ./255;
TumColors.Extended.E7 =           [249  186  0] ./255;
TumColors.Extended.E8 =           [214  76  19] ./255;
TumColors.Extended.E9 =           [196  7  27] ./255;
TumColors.Extended.E10 =          [156  13  22] ./255;    

TumColors.Colormap.Primary = cell2mat(struct2cell(TumColors.primary));
TumColors.Colormap.Secondary = cell2mat(struct2cell(TumColors.secondary));
TumColors.Colormap.Accent = cell2mat(struct2cell(TumColors.accent));
TumColors.Colormap.Custom = [TumColors.secondary.DarkBlue; TumColors.secondary.LightBlue; TumColors.primary.Blue;...
    TumColors.accent.DarkBlue; TumColors.accent.LightBlue; TumColors.accent.Orange; TumColors.accent.Green];

end

