function result = generate_m400_scan_rotate_kmz(cfg)
%GENERATE_M400_SCAN_ROTATE_KMZ 扫描航线加定距停转点，每点顺序执行 4 次 90 度旋转。
% cfg.rotationSpacingM 必须为正数；其余配置与 generate_m400_scan_kmz 相同。
if nargin < 1 || ~isstruct(cfg) || ~isfield(cfg,'rotationSpacingM') || ...
        ~isnumeric(cfg.rotationSpacingM) || ~isscalar(cfg.rotationSpacingM) || ...
        ~isfinite(cfg.rotationSpacingM) || cfg.rotationSpacingM <= 0
    error('请在 cfg.rotationSpacingM 中设置正的停转间距（米）。');
end
result = generate_m400_scan_kmz(cfg);
end
