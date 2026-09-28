% 多高度扫描 + 定距停下自旋 360 度。只生成 KMZ 和预览，不连接飞机。
toolDir = fileparts(mfilename('fullpath'));
addpath(toolDir);

% ======================== 常改参数 ========================
cfg = struct;
cfg.heightsM = [50];       % 各高度层，相对起飞点，米
cfg.scanSpacingM = 10;           % 平行扫描线间距，米
cfg.directionDeg = 45;           % 扫描方向：0=南北，90=东西
cfg.flightSpeedMps = 5;         % 扫描速度，m/s
cfg.rotationSpacingM = 40;      % 每层沿实际航线累计距离，每隔多少米停下自旋一次

cfg.updateRegion = false;       % 只有需要重新在地图点选区域时才设为 true
cfg.regionLatLon = [];          % 或直接填写 N×2 [纬度 经度]；[] 复用已保存区域
cfg.mapCenterLatLon = [22.60255 113.99172];
cfg.mapSpanDeg = [0.01 0.01];
cfg.mapBasemap = 'satellite';   % 无网络可改为 'landcover'
cfg.edgeInsetM = 2;
cfg.takeoffSecurityHeightM = 40;
cfg.globalTransitionalSpeedMps = 10;
cfg.rthHeightM = 50;
cfg.finishAction = 'goHome';
cfg.rcLostAction = 'goBack';

cfg.showPreview = true;
cfg.savePreviewImages = true;
% ==========================================================

cfg.outputDir = fullfile(toolDir, 'waypoint_output');
cfg.outputName = ['m400_scan_rotate_' char(datetime('now','Format','yyyyMMdd_HHmmss_SSS'))];

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

result = generate_m400_scan_rotate_kmz(cfg);
fprintf('KMZ：%s\n停转点：%d 个\n', result.kmzPath, numel(result.rotationIndices));
if cfg.showPreview && cfg.savePreviewImages
    fprintf('2D 地图：%s\n3D 航线：%s\n', ...
        result.previewPaths.map2D, result.previewPaths.route3D);
end
