%class ClockFactor, see Doxygen page for details
%at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
%
%-------Constructors-------
%ClockFactor(size_t keyC1, size_t keyC2, size_t keyD1, size_t keyD2, double dt, Base model)
%
classdef ClockFactor < gtsam.NoiseModelFactor
  properties
    ptr_gtsam_gnssClockFactor = 0
  end
  methods
    function obj = ClockFactor(varargin)
      if (nargin == 2 || (nargin == 3 && strcmp(varargin{3}, 'void'))) && isa(varargin{1}, 'uint64') && varargin{1} == uint64(5139824614673773682)
        if nargin == 2
          my_ptr = varargin{2};
        else
          my_ptr = gtsam_gnss_wrapper(61, varargin{2});
        end
        base_ptr = gtsam_gnss_wrapper(60, my_ptr);
      elseif nargin == 6 && isa(varargin{1},'numeric') && isa(varargin{2},'numeric') && isa(varargin{3},'numeric') && isa(varargin{4},'numeric') && isa(varargin{5},'double') && isa(varargin{6},'gtsam.noiseModel.Base')
        [ my_ptr, base_ptr ] = gtsam_gnss_wrapper(62, varargin{1}, varargin{2}, varargin{3}, varargin{4}, varargin{5}, varargin{6});
      else
        error('Arguments do not match any overload of gtsam_gnss.ClockFactor constructor');
      end
      obj = obj@gtsam.NoiseModelFactor(uint64(5139824614673773682), base_ptr);
      obj.ptr_gtsam_gnssClockFactor = my_ptr;
    end

    function delete(obj)
      gtsam_gnss_wrapper(63, obj.ptr_gtsam_gnssClockFactor);
    end

    function display(obj), obj.print(''); end
    %DISPLAY Calls print on the object
    function disp(obj), obj.display; end
    %DISP Calls print on the object
  end

  methods(Static = true)
  end
end
