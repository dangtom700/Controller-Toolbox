classdef TelemetryLogger < handle
% TELEMETRYLOGGER  Per-tick CSV writer + IAE/ISE/E_valve accumulator.
%   Emits the exact 23-column schema of the C++ telemetry_logger so the output
%   flows through the existing Python report pipeline unchanged:
%     t,y1,y2,y3,u1,u2,u3,du1,du2,du3,ref_y1,ref_y2,ref_y3,e1,e2,e3,
%     IAE_y1,IAE_y2,IAE_y3,ISE_y1,ISE_y2,ISE_y3,E_valve
%   Error convention matches the C++ logger: e = ref - y, dt = Ts each step.

    properties (Access = private)
        fid = -1
        IAE = [0 0 0]
        ISE = [0 0 0]
        Ev  = 0
        Ts  = 1.0
    end

    methods
        function obj = TelemetryLogger(filepath)
            obj.fid = fopen(filepath, 'w');
            if obj.fid < 0
                error('boiler:log', 'Cannot open log file: %s', filepath);
            end
            fprintf(obj.fid, ['t,y1,y2,y3,u1,u2,u3,du1,du2,du3,', ...
                'ref_y1,ref_y2,ref_y3,e1,e2,e3,', ...
                'IAE_y1,IAE_y2,IAE_y3,ISE_y1,ISE_y2,ISE_y3,E_valve\n']);
        end

        function log(obj, t, y, u, du, ref)
            dt = obj.Ts;
            e  = ref(:) - y(:);
            obj.IAE = obj.IAE + abs(e).' * dt;
            obj.ISE = obj.ISE + (e.^2).' * dt;
            obj.Ev  = obj.Ev  + sum(du(:).^2) * dt;
            fprintf(obj.fid, ['%.6e,%.6e,%.6e,%.6e,%.6e,%.6e,%.6e,', ...
                '%.6e,%.6e,%.6e,%.6e,%.6e,%.6e,%.6e,%.6e,%.6e,', ...
                '%.6e,%.6e,%.6e,%.6e,%.6e,%.6e,%.6e\n'], ...
                t, y(1), y(2), y(3), u(1), u(2), u(3), ...
                du(1), du(2), du(3), ref(1), ref(2), ref(3), ...
                e(1), e(2), e(3), ...
                obj.IAE(1), obj.IAE(2), obj.IAE(3), ...
                obj.ISE(1), obj.ISE(2), obj.ISE(3), obj.Ev);
        end

        function [iae, ise, ev] = finals(obj)
            iae = obj.IAE;  ise = obj.ISE;  ev = obj.Ev;
        end

        function close(obj)
            if obj.fid >= 0, fclose(obj.fid); obj.fid = -1; end
        end

        function delete(obj)
            obj.close();
        end
    end
end
