%class TDCPFactorWithClock, see Doxygen page for details
%at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
%
%-------Constructors-------
%TDCPFactorWithClock(size_t keyX1, size_t keyX2, size_t keyC1, size_t keyC2, Vector losvec, Vector L1, Vector L2, int sigtype, Vector orgx1, Vector orgx2, Base model)
%
classdef TDCPFactorWithClock < gtsam.NoiseModelFactor
  properties
    ptr_gtsam_gnssTDCPFactorWithClock = 0
  end
  methods
    function obj = TDCPFactorWithClock(varargin)
      if (nargin == 2 || (nargin == 3 && strcmp(varargin{3}, 'void'))) && isa(varargin{1}, 'uint64') && varargin{1} == uint64(5139824614673773682)
        if nargin == 2
          my_ptr = varargin{2};
        else
          my_ptr = gtsam_gnss_wrapper(49, varargin{2});
        end
        base_ptr = gtsam_gnss_wrapper(48, my_ptr);
      elseif nargin == 11 && isa(varargin{1},'numeric') && isa(varargin{2},'numeric') && isa(varargin{3},'numeric') && isa(varargin{4},'numeric') && isa(varargin{5},'double') && isa(varargin{6},'double') && isa(varargin{7},'double') && isa(varargin{8},'numeric') && isa(varargin{9},'double') && isa(varargin{10},'double') && isa(varargin{11},'gtsam.noiseModel.Base')
        [ my_ptr, base_ptr ] = gtsam_gnss_wrapper(50, varargin{1}, varargin{2}, varargin{3}, varargin{4}, varargin{5}, varargin{6}, varargin{7}, varargin{8}, varargin{9}, varargin{10}, varargin{11});
      else
        error('Arguments do not match any overload of gtsam_gnss.TDCPFactorWithClock constructor');
      end
      obj = obj@gtsam.NoiseModelFactor(uint64(5139824614673773682), base_ptr);
      obj.ptr_gtsam_gnssTDCPFactorWithClock = my_ptr;
    end

    function delete(obj)
      gtsam_gnss_wrapper(51, obj.ptr_gtsam_gnssTDCPFactorWithClock);
    end

    function display(obj), obj.print(''); end
    %DISPLAY Calls print on the object
    function disp(obj), obj.display; end
    %DISP Calls print on the object
  end

  methods(Static = true)
  end
end
