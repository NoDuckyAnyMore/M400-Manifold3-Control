function result = generate_m400_scan_kmz(cfg)
%GENERATE_M400_SCAN_KMZ 多高度层区域扫描 -> DJI WPML KMZ (DJI Pilot 2 导入).
%
% 用法：编辑下面「用户配置」，然后在 MATLAB 中运行：
%   addpath('D:\M400-Manifold3-Control\tools')
%   result = generate_m400_scan_kmz;
%
% 也可用 cfg 结构体覆盖默认值，便于自动化测试。
% 本程序只生成文件，不连接/控制飞机。首次必须填写 NaN/[] 参数。
% 经纬度输入顺序为 [纬度, 经度]；WPML 写出顺序为 经度,纬度。
% WPML: https://developer.dji.com/doc/cloud-api-tutorial/cn/api-reference/dji-wpml/

% =========================== 用户配置 ===========================
defaults.updateRegion = false;       % true: 在地图上重新点区域，回车结束并保存；false: 不再点
defaults.regionLatLon = [];          % 可直接填 N×2 [纬度 经度]；[] 时读取上次点击保存的区域
defaults.mapCenterLatLon = [22.60255, 113.99172]; % 仅供点选地图初始视野，不是航点
defaults.mapSpanDeg = [0.01, 0.01];  % 地图视野 [纬度跨度 经度跨度]
defaults.mapBasemap = 'satellite';   % 在线底图需联网；离线可改为 'landcover'

defaults.heightsM = [];              % 必填，如 [20 30 40]；相对起飞点高度，按给定顺序飞
defaults.scanSpacingM = NaN;         % 必填，平行扫描线间距，米
defaults.directionDeg = NaN;         % 必填，扫描线航向：0=南北，90=东西，顺时针自北
defaults.flightSpeedMps = NaN;       % 必填，航线速度，WPML 范围 1~15 m/s
defaults.globalTransitionalSpeedMps = 10; % 飞往首航点等航线过渡速度，m/s；与扫描速度分开
defaults.takeoffSecurityHeightM = 40; % 安全起飞高度，相对起飞点，米
defaults.rthHeightM = 50;           % 返航高度，相对起飞点，米
defaults.edgeInsetM = 2;            % 区域边界内缩，米；不需内缩可设 0
defaults.finishAction = 'goHome';    % goHome | noAction | autoLand | gotoFirstWaypoint
defaults.rcLostAction = 'goBack';    % goBack | landing | hover；失控即退出任务
defaults.author = 'M400 Research';
defaults.payloadEnumValue = [];     % 无负载、无负载动作：不写 payloadInfo
defaults.payloadPositionIndex = 0;  % 仅在设置 payloadEnumValue 时使用
defaults.wpmlNamespace = 'http://www.dji.com/wpmz/1.0.6';
defaults.outputName = 'm400_multilayer_scan'; % 不覆盖同名文件，改名或移走旧文件再生成
defaults.outputDir = fullfile(fileparts(mfilename('fullpath')), 'waypoint_output');
defaults.showPreview = true;        % 显示二维地图叠加和三维航线
defaults.savePreviewImages = true;  % 将两张预览图保存为 PNG；showPreview=false 时不生成
% ================================================================

if nargin < 1 || isempty(cfg)
    cfg = defaults;
else
    keys = fieldnames(defaults);
    for k = 1:numel(keys)
        if ~isfield(cfg, keys{k}), cfg.(keys{k}) = defaults.(keys{k}); end
    end
end

cacheFile = fullfile(fileparts(mfilename('fullpath')), 'm400_scan_region.mat');
if cfg.updateRegion
    regionLatLon = pickRegion(cfg);
elseif ~isempty(cfg.regionLatLon)
    regionLatLon = cfg.regionLatLon;
elseif isfile(cacheFile)
    saved = load(cacheFile, 'regionLatLon');
    regionLatLon = saved.regionLatLon;
else
    error('尚无区域：将 updateRegion 设为 true 点选，或填写 regionLatLon。');
end

validateConfig(cfg, regionLatLon);
if cfg.updateRegion
    save(cacheFile, 'regionLatLon');
end
[polygonENU, geo] = llToENU(regionLatLon);
if cfg.edgeInsetM > 0
    insetShape = polybuffer(polyshape(polygonENU(:,1), polygonENU(:,2)), -cfg.edgeInsetM);
    if area(insetShape) <= 0 || numel(regions(insetShape)) ~= 1
        error('边界内缩后区域为空或分裂；请减小 edgeInsetM。');
    end
    [px, py] = boundary(insetShape);
    if any(~isfinite(px)) || any(~isfinite(py))
        error('当前仅支持无洞、单个闭合区域；请调整区域或内缩距离。');
    end
    polygonENU = [px(:), py(:)];
end
if max(vecnorm(polygonENU,2,2)) > 5000
    error('区域距中心超过 5 km；当前局部 WGS84 平面近似不适用。');
end

basePath = buildScanPath(polygonENU, cfg.scanSpacingM, cfg.directionDeg);
if size(basePath,1) < 2, error('无法在该区域生成至少两个航点。'); end
pathENU = zeros(0,2);
pathHeight = zeros(0,1);
layerIndex = zeros(0,1);
for layer = 1:numel(cfg.heightsM)
    layerPath = basePath;
    if mod(layer,2) == 0, layerPath = flipud(layerPath); end
    pathENU = [pathENU; layerPath]; %#ok<AGROW>
    pathHeight = [pathHeight; repmat(cfg.heightsM(layer),size(layerPath,1),1)]; %#ok<AGROW>
    layerIndex = [layerIndex; repmat(layer,size(layerPath,1),1)]; %#ok<AGROW>
end
if size(pathENU,1) > 65535, error('总航点数超过 WPML 的 65535 上限。'); end
pathLatLon = enuToLL(pathENU, geo);

if ~isfolder(cfg.outputDir), mkdir(cfg.outputDir); end
kmzPath = fullfile(cfg.outputDir, [cfg.outputName '.kmz']);
if isfile(kmzPath), error('目标 KMZ 已存在，避免覆盖：%s', kmzPath); end
previewPaths = struct('map2D','','route3D','');
if cfg.showPreview
    [fig2D,fig3D] = previewRoute(regionLatLon, pathLatLon, pathENU, pathHeight, layerIndex, cfg);
    if cfg.savePreviewImages
        previewPaths.map2D = fullfile(cfg.outputDir,[cfg.outputName '_map2d.png']);
        previewPaths.route3D = fullfile(cfg.outputDir,[cfg.outputName '_route3d.png']);
        if isfile(previewPaths.map2D) || isfile(previewPaths.route3D)
            error('预览图片已存在；请修改 outputName 或移走旧图片。');
        end
        exportgraphics(fig2D,previewPaths.map2D,'Resolution',200);
        exportgraphics(fig3D,previewPaths.route3D,'Resolution',200);
    end
end

stageRoot = tempname;
mkdir(fullfile(stageRoot, 'wpmz'));
stageCleanup = onCleanup(@() removeStage(stageRoot));
templatePath = fullfile(stageRoot, 'wpmz', 'template.kml');
waylinesPath = fullfile(stageRoot, 'wpmz', 'waylines.wpml');
writeWpml(templatePath, cfg, pathLatLon, pathHeight, false);
writeWpml(waylinesPath, cfg, pathLatLon, pathHeight, true);
zipPath = fullfile(stageRoot, [cfg.outputName '.zip']);
zip(zipPath, {'wpmz/template.kml','wpmz/waylines.wpml'}, stageRoot);
movefile(zipPath, kmzPath);
clear stageCleanup;

result = struct('kmzPath',kmzPath,'waypointCount',size(pathLatLon,1), ...
    'layerCount',numel(cfg.heightsM),'regionLatLon',regionLatLon, ...
    'waypointLatLon',pathLatLon,'waypointHeightM',pathHeight, ...
    'layerIndex',layerIndex,'previewPaths',previewPaths);
fprintf('已生成 %s：%d 个航点，%d 个高度层。先在 Pilot 2 导入预览，勿直接起飞。\n', ...
    kmzPath, result.waypointCount, result.layerCount);
end

function region = pickRegion(cfg)
fig = figure('Name','点选区域：按顺时针点顶点，回车结束','NumberTitle','off');
gx = geoaxes(fig);
geobasemap(gx, cfg.mapBasemap);
geolimits(gx, cfg.mapCenterLatLon(1)+[-1 1]*cfg.mapSpanDeg(1)/2, ...
    cfg.mapCenterLatLon(2)+[-1 1]*cfg.mapSpanDeg(2)/2);
title(gx, '顺时针点选至少 3 个区域顶点，回车结束');
% MATLAB 的 ginput 在 geoaxes 上依次返回纬度、经度。
[lat,lon] = ginput;
region = [lat(:),lon(:)];
if size(region,1) < 3
    error('少于 3 个点；旧区域缓存未更改。');
end
hold(gx,'on');
geoplot(gx,[lat(:);lat(1)],[lon(:);lon(1)],'r-','LineWidth',2);
end

function validateConfig(c, ll)
if ~isnumeric(ll) || size(ll,2) ~= 2 || size(ll,1) < 3 || ...
        any(~isfinite(ll(:))) || any(abs(ll(:,1)) > 90) || any(abs(ll(:,2)) > 180)
    error('regionLatLon 必须为至少 3 行的 [纬度 经度] 有效数值矩阵。');
end
if size(ll,1)>3 && norm(ll(1,:)-ll(end,:)) < 1e-10, ll(end,:) = []; end
if ~isnumeric(c.heightsM) || isempty(c.heightsM) || ...
        any(~isfinite(c.heightsM(:))) || any(c.heightsM(:) <= 0)
    error('请填写正数数组 heightsM（相对起飞点高度，米）。');
end
checkScalar(c.scanSpacingM,'scanSpacingM',0,Inf);
checkScalar(c.directionDeg,'directionDeg',0,360);
checkScalar(c.flightSpeedMps,'flightSpeedMps',1,15);
checkScalar(c.globalTransitionalSpeedMps,'globalTransitionalSpeedMps',1,15);
checkScalar(c.takeoffSecurityHeightM,'takeoffSecurityHeightM',1.2,1500);
checkScalar(c.rthHeightM,'rthHeightM',1.2,1500);
checkScalar(c.edgeInsetM,'edgeInsetM',0,Inf);
if ~ismember(c.finishAction,{'goHome','noAction','autoLand','gotoFirstWaypoint'})
    error('finishAction 不属于 WPML 允许值。');
end
if ~ismember(c.rcLostAction,{'goBack','landing','hover'})
    error('rcLostAction 不属于 WPML 允许值。');
end
if isempty(regexp(c.outputName,'^[A-Za-z0-9_-]+$','once'))
    error('outputName 只能由英文、数字、下划线和连字符组成。');
end
if ~isempty(c.payloadEnumValue)
    checkScalar(c.payloadEnumValue,'payloadEnumValue',1,65535);
    checkScalar(c.payloadPositionIndex,'payloadPositionIndex',0,7);
end
[xy,~] = llToENU(ll);
pg = polyshape(xy(:,1),xy(:,2));
if area(pg) < 1 || numel(regions(pg)) ~= 1
    error('区域无效、自交或面积小于 1 m^2；请重新选点。');
end
if size(pg.Vertices,1) ~= size(xy,1)
    error('区域可能自交/重复顶点；请给出简单闭合多边形。');
end
end

function checkScalar(value,name,lo,hi)
if ~isnumeric(value) || ~isscalar(value) || ~isfinite(value) || value<lo || value>hi
    error('%s 必须是 [%g,%g] 范围内的有限标量。',name,lo,hi);
end
end

function [en, geo] = llToENU(ll)
if size(ll,1)>3 && norm(ll(1,:)-ll(end,:))<1e-10, ll(end,:)=[]; end
geo.lat0 = mean(ll(:,1));
geo.lon0 = mean(ll(:,2));
a=6378137; e2=6.69437999014e-3; phi=deg2rad(geo.lat0);
geo.eastPerDeg = (pi/180)*a*cos(phi)/sqrt(1-e2*sin(phi)^2);
geo.northPerDeg = (pi/180)*a*(1-e2)/(1-e2*sin(phi)^2)^(3/2);
en = [(ll(:,2)-geo.lon0)*geo.eastPerDeg, ...
      (ll(:,1)-geo.lat0)*geo.northPerDeg];
end

function ll = enuToLL(en,geo)
ll = [geo.lat0+en(:,2)/geo.northPerDeg, ...
      geo.lon0+en(:,1)/geo.eastPerDeg];
end

function path = buildScanPath(poly, spacing, bearing)
theta=deg2rad(bearing);
u=[sin(theta),cos(theta)]; v=[cos(theta),-sin(theta)];
s=poly*u'; t=poly*v';
t0=min(t); t1=max(t);
levels=(t0+spacing/2):spacing:t1;
if isempty(levels), levels=(t0+t1)/2; end
path=zeros(0,2);
for row=1:numel(levels)
    tv=levels(row);
    cuts=[];
    for j=1:size(poly,1)
        k=mod(j,size(poly,1))+1;
        if (t(j)<=tv && t(k)>tv) || (t(k)<=tv && t(j)>tv)
            cuts(end+1)=s(j)+(tv-t(j))*(s(k)-s(j))/(t(k)-t(j)); %#ok<AGROW>
        end
    end
    cuts=sort(cuts);
    if mod(numel(cuts),2)~=0, error('扫描线与多边形交点数异常；请检查区域形状。'); end
    pairs=zeros(0,2);
    for j=1:2:numel(cuts)
        mid=(cuts(j)+cuts(j+1))/2;
        p=mid*u+tv*v;
        [inside,on]=inpolygon(p(1),p(2),poly(:,1),poly(:,2));
        if (inside||on) && cuts(j+1)-cuts(j)>0.05
            pairs(end+1,:)=[cuts(j),cuts(j+1)]; %#ok<AGROW>
        end
    end
    if mod(row,2)==0, pairs=flipud(pairs); end
    for j=1:size(pairs,1)
        if mod(row,2)==1, a=pairs(j,1); b=pairs(j,2);
        else, a=pairs(j,2); b=pairs(j,1); end
        first=a*u+tv*v; last=b*u+tv*v;
        if isempty(path)
            path=[first;last];
        else
            connector=insideConnector(path(end,:),first,poly);
            path=[path;connector(2:end,:);last]; %#ok<AGROW>
        end
    end
end
if isempty(path), error('扫描间距过大或区域过窄，未生成航线。'); end
end

function route = insideConnector(p,q,poly)
if norm(p-q)<0.01, route=[p;q]; return; end
if segmentInside(p,q,poly), route=[p;q]; return; end
nodes=[p;q;poly];
n=size(nodes,1);
cost=inf(n); cost(1:n+1:end)=0;
for i=1:n
    for j=i+1:n
        if segmentInside(nodes(i,:),nodes(j,:),poly)
            cost(i,j)=norm(nodes(i,:)-nodes(j,:)); cost(j,i)=cost(i,j);
        end
    end
end
dist=inf(n,1); prev=zeros(n,1); used=false(n,1); dist(1)=0;
for iter=1:n
    remaining=dist; remaining(used)=inf;
    [best,i]=min(remaining);
    if ~isfinite(best) || i==2, break; end
    used(i)=true;
    for j=1:n
        alt=dist(i)+cost(i,j);
        if ~used(j) && alt<dist(j), dist(j)=alt; prev(j)=i; end
    end
end
if ~isfinite(dist(2)), error('凹区域内无法连接扫描段；请修改区域或边界内缩量。'); end
idx=2;
while idx(1)~=1, idx=[prev(idx(1)),idx]; end %#ok<AGROW>
route=nodes(idx,:);
end

function yes = segmentInside(p,q,poly)
r=q-p;
if norm(r)<1e-8
    [inside,on]=inpolygon(p(1),p(2),poly(:,1),poly(:,2));
    yes=inside||on; return;
end
params=[0,1];
for j=1:size(poly,1)
    a=poly(j,:); b=poly(mod(j,size(poly,1))+1,:);
    e=b-a; den=cross2(r,e);
    if abs(den)<1e-10
        if abs(cross2(a-p,r))<1e-8
            params=[params,dot(a-p,r)/dot(r,r),dot(b-p,r)/dot(r,r)]; %#ok<AGROW>
        end
    else
        z=cross2(a-p,e)/den; w=cross2(a-p,r)/den;
        if z>=-1e-9 && z<=1+1e-9 && w>=-1e-9 && w<=1+1e-9
            params(end+1)=z; %#ok<AGROW>
        end
    end
end
params=unique(max(0,min(1,params)));
yes=true;
for j=1:numel(params)-1
    z=(params(j)+params(j+1))/2;
    sample=p+z*r;
    [inside,on]=inpolygon(sample(1),sample(2),poly(:,1),poly(:,2));
    if ~(inside||on), yes=false; return; end
end
end

function z = cross2(a,b)
z=a(1)*b(2)-a(2)*b(1);
end

function [fig2D,fig3D] = previewRoute(regionLL,routeLL,routeEN,heights,layers,cfg)
fig2D=figure('Name','M400 多高度层扫描航线（二维）','NumberTitle','off');
gx=geoaxes;
geobasemap(gx,cfg.mapBasemap);
hold(gx,'on');
geoplot(gx,[regionLL(:,1);regionLL(1,1)], ...
    [regionLL(:,2);regionLL(1,2)],'k-','LineWidth',1.5);
palette=lines(max(layers));
for k=1:max(layers)
    idx=find(layers==k);
    geoplot(gx,routeLL(idx,1),routeLL(idx,2),'-o', ...
        'Color',palette(k,:),'MarkerSize',3);
end
title(gx,'二维预览：黑线=区域；彩线=各高度层（跨层连接看三维图）');
fig3D=figure('Name','M400 多高度层扫描航线（三维）','NumberTitle','off');
hold on;
for k=1:max(layers)
    idx=find(layers==k);
    plot3(routeEN(idx,1),routeEN(idx,2),heights(idx),'-o', ...
        'Color',palette(k,:),'MarkerSize',3, ...
        'DisplayName',sprintf('第 %d 层：%g m',k,heights(idx(1))));
    if k>1
        p=idx(1)-1;
        plot3(routeEN([p idx(1)],1),routeEN([p idx(1)],2), ...
            heights([p idx(1)]),'k--','HandleVisibility','off');
    end
end
plot3(routeEN(1,1),routeEN(1,2),heights(1),'go', ...
    'MarkerFaceColor','g','DisplayName','起点');
plot3(routeEN(end,1),routeEN(end,2),heights(end),'rs', ...
    'MarkerFaceColor','r','DisplayName','终点');
grid on; axis equal; xlabel('东 / m'); ylabel('北 / m');
zlabel('相对起飞点高度 / m'); title('三维航点顺序');
view(35,25); legend('Location','best');
end

function writeWpml(path,cfg,ll,h,executable)
fid=fopen(path,'w','n','UTF-8');
if fid<0, error('无法写入 XML：%s',path); end
guard=onCleanup(@() fclose(fid));
fprintf(fid,'<?xml version="1.0" encoding="UTF-8"?>\n');
fprintf(fid,'<kml xmlns="http://www.opengis.net/kml/2.2" xmlns:wpml="%s">\n', ...
    xmlEscape(cfg.wpmlNamespace));
fprintf(fid,'  <Document>\n');
if ~executable
    tag(fid,4,'author',xmlEscape(cfg.author));
    nowMs=sprintf('%.0f',posixtime(datetime('now','TimeZone','UTC'))*1000);
    tag(fid,4,'createTime',nowMs);
    tag(fid,4,'updateTime',nowMs);
end
fprintf(fid,'    <wpml:missionConfig>\n');
tag(fid,6,'flyToWaylineMode','safely');
tag(fid,6,'finishAction',cfg.finishAction);
tag(fid,6,'exitOnRCLost','executeLostAction');
tag(fid,6,'executeRCLostAction',cfg.rcLostAction);
tag(fid,6,'takeOffSecurityHeight',num(cfg.takeoffSecurityHeightM));
tag(fid,6,'globalTransitionalSpeed',num(cfg.globalTransitionalSpeedMps));
tag(fid,6,'globalRTHHeight',num(cfg.rthHeightM));
fprintf(fid,'      <wpml:droneInfo>\n');
tag(fid,8,'droneEnumValue','103'); % DJI 官方机型表：Matrice 400 = 103/0
tag(fid,8,'droneSubEnumValue','0');
fprintf(fid,'      </wpml:droneInfo>\n');
if ~isempty(cfg.payloadEnumValue)
    fprintf(fid,'      <wpml:payloadInfo>\n');
    tag(fid,8,'payloadEnumValue',num(cfg.payloadEnumValue));
    tag(fid,8,'payloadPositionIndex',num(cfg.payloadPositionIndex));
    fprintf(fid,'      </wpml:payloadInfo>\n');
end
fprintf(fid,'    </wpml:missionConfig>\n');
fprintf(fid,'    <Folder>\n');
if executable
    tag(fid,6,'templateId','0');
    tag(fid,6,'executeHeightMode','relativeToStartPoint');
    tag(fid,6,'waylineId','0');
    tag(fid,6,'autoFlightSpeed',num(cfg.flightSpeedMps));
else
    tag(fid,6,'templateType','waypoint');
    tag(fid,6,'templateId','0');
    fprintf(fid,'      <wpml:waylineCoordinateSysParam>\n');
    tag(fid,8,'coordinateMode','WGS84');
    tag(fid,8,'heightMode','relativeToStartPoint');
    tag(fid,8,'positioningType','GPS');
    fprintf(fid,'      </wpml:waylineCoordinateSysParam>\n');
    tag(fid,6,'autoFlightSpeed',num(cfg.flightSpeedMps));
    tag(fid,6,'globalHeight',num(h(1)));
    tag(fid,6,'gimbalPitchMode','manual');
    fprintf(fid,'      <wpml:globalWaypointHeadingParam>\n');
    writeHeading(fid,8);
    fprintf(fid,'      </wpml:globalWaypointHeadingParam>\n');
    tag(fid,6,'globalWaypointTurnMode','toPointAndStopWithDiscontinuityCurvature');
    tag(fid,6,'globalUseStraightLine','1');
end
for i=1:size(ll,1)
    fprintf(fid,'      <Placemark>\n');
    fprintf(fid,'        <Point><coordinates>%.10f,%.10f</coordinates></Point>\n',ll(i,2),ll(i,1));
    tag(fid,8,'index',sprintf('%d',i-1));
    if executable
        tag(fid,8,'executeHeight',num(h(i)));
        tag(fid,8,'waypointSpeed',num(cfg.flightSpeedMps));
        fprintf(fid,'        <wpml:waypointHeadingParam>\n');
        writeHeading(fid,10);
        fprintf(fid,'        </wpml:waypointHeadingParam>\n');
        fprintf(fid,'        <wpml:waypointTurnParam>\n');
        tag(fid,10,'waypointTurnMode','toPointAndStopWithDiscontinuityCurvature');
        tag(fid,10,'waypointTurnDampingDist','0');
        fprintf(fid,'        </wpml:waypointTurnParam>\n');
        tag(fid,8,'useStraightLine','1');
    else
        % 官方 WPML：相对起飞点模式下 ellipsoidHeight 与 height 数值相同。
        tag(fid,8,'ellipsoidHeight',num(h(i)));
        tag(fid,8,'height',num(h(i)));
        tag(fid,8,'useGlobalHeight','0');
        tag(fid,8,'useGlobalSpeed','1');
        tag(fid,8,'useGlobalHeadingParam','1');
        tag(fid,8,'useGlobalTurnParam','1');
        tag(fid,8,'useStraightLine','1');
    end
    fprintf(fid,'      </Placemark>\n');
end
fprintf(fid,'    </Folder>\n  </Document>\n</kml>\n');
clear guard;
end

function writeHeading(fid,indent)
tag(fid,indent,'waypointHeadingMode','followWayline');
tag(fid,indent,'waypointHeadingAngle','0');
tag(fid,indent,'waypointPoiPoint','0.000000,0.000000,0.000000');
end

function tag(fid,indent,name,value)
fprintf(fid,'%s<wpml:%s>%s</wpml:%s>\n',repmat(' ',1,indent),name,value,name);
end

function value = num(x)
value=sprintf('%.8f',x);
end

function out = xmlEscape(value)
out=char(value);
out=strrep(out,'&','&amp;');
out=strrep(out,'<','&lt;');
out=strrep(out,'>','&gt;');
out=strrep(out,'"','&quot;');
out=strrep(out,'''','&apos;');
end

function removeStage(path)
% tempname 生成的专属临时目录，确认在系统 tempdir 下后才清理。
if startsWith(path,tempdir) && isfolder(path)
    rmdir(path,'s');
end
end
