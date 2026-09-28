% M400 多高度层扫描航线入口：在 MATLAB 中打开本文件，修改下面参数，点击“运行”。
% 只生成 KMZ 和预览图，不连接飞机；高度均相对起飞点。

toolDir = fileparts(mfilename('fullpath'));
addpath(toolDir);

% ======================== 常改参数 ========================
cfg = struct;
cfg.heightsM = [55 60 65];       % 示例：依次扫描的高度层，米；按任务修改
cfg.scanSpacingM = 10;           % 相邻扫描线间距，米
cfg.directionDeg = 45;           % 扫描方向：0=南北，90=东西，顺时针自北
cfg.flightSpeedMps = 5;         % 扫描飞行速度，m/s；与过渡速度分开

cfg.updateRegion = false;       % 只有想重新在地图上点选区域时，才改为 true
cfg.regionLatLon = [];          % 可直接填 N×2 [纬度 经度]；[] 则使用上次保存的区域
cfg.mapCenterLatLon = [22.60255 113.99172]; % 地图点选时的初始视野中心
cfg.mapSpanDeg = [0.01 0.01];  % 地图初始视野跨度 [纬度 经度]
cfg.mapBasemap = 'satellite';   % 卫星地图需要联网
cfg.edgeInsetM = 2;            % 扫描线离区域边界的内缩距离，米

cfg.takeoffSecurityHeightM = 40;
cfg.globalTransitionalSpeedMps = 10;
cfg.rthHeightM = 50;
cfg.finishAction = 'goHome';    % 航线结束返航
cfg.rcLostAction = 'goBack';    % 失控退出任务并返航

cfg.showPreview = true;        % 弹出 2D 地图和 3D 航线窗口
cfg.savePreviewImages = true;  % 同时保存两张 PNG
% ==========================================================

cfg.outputDir = fullfile(toolDir, 'waypoint_output');
cfg.outputName = ['m400_scan_' char(datetime('now','Format','yyyyMMdd_HHmmss_SSS'))];

% 首次没有保存的区域时才询问；以后默认复用，不会每次弹出点选地图。
cacheFile = fullfile(toolDir, 'm400_scan_region.mat');
if isempty(cfg.regionLatLon) && ~cfg.updateRegion && ~isfile(cacheFile)
    choice = questdlg('还没有保存的区域。现在打开地图点选吗？', ...
        '首次选择扫描区域', '现在点选', '取消', '现在点选');
    if ~strcmp(choice, '现在点选')
        disp('未选择区域，未生成 KMZ。');
        return;
    end
    cfg.updateRegion = true;
end

result = generate_m400_scan_kmz(cfg);
fprintf('KMZ：%s\n', result.kmzPath);
if cfg.showPreview && cfg.savePreviewImages
    fprintf('2D 地图：%s\n3D 航线：%s\n', ...
        result.previewPaths.map2D, result.previewPaths.route3D);
end
