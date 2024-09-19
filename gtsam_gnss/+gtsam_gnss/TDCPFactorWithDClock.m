%class TDCPFactorWithDClock, see Doxygen page for details
%at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
%
%-------Constructors-------
%TDCPFactorWithDClock(size_t keyX1, size_t keyX2, size_t keyD1, size_t keyD2, Vector losvec, Vector L1, Vector L2, Vector orgx1, Vector orgx2, Base model)
%
classdef TDCPFactorWithDClock < gtsam.NoiseModelFactor
  properties
    ptr_gtsam_gnssTDCPFactorWithDClock = 0
  end
  methods
    function obj = TDCPFactorWithDClock(varargin)
      if (nargin == 2 || (nargin == 3 && strcmp(varargin{3}, 'void'))) && isa(varargin{1}, 'uint64') && varargin{1} == uint64(5139824614673773682)
        if nargin == 2
          my_ptr = varargin{2};
        else
          my_ptr = gtsam_gnss_wrapper(53, varargin{2});
        end
        base_ptr = gtsam_gnss_wrapper(52, my_ptr);
      elseif nargin == 10 && isa(varargin{1},'numeric') && isa(varargin{2},'numeric') && isa(varargin{3},'numeric') && isa(varargin{4},'numeric') && isa(varargin{5},'double') && isa(varargin{6},'double') && isa(varargin{7},'double') && isa(varargin{8},'double') && isa(varargin{9},'double') && isa(varargin{10},'gtsam.noiseModel.Base')
        [ my_ptr, base_ptr ] = gtsam_gnss_wrapper(54, varargin{1}, varargin{2}, varargin{3}, varargin{4}, varargin{5}, varargin{6}, varargin{7}, varargin{8}, varargin{9}, varargin{10});
      else
        error('Arguments do not match any overload of gtsam_gnss.TDCPFactorWithDClock constructor');
      end
      obj = obj@gtsam.NoiseModelFactor(uint64(5139824614673773682), base_ptr);
      obj.ptr_gtsam_gnssTDCPFactorWithDClock = my_ptr;
    end

    function delete(obj)
      gtsam_gnss_wrapper(55, obj.ptr_gtsam_gnssTDCPFactorWithDClock);
    end

    function display(obj), obj.print(''); end
    %DISPLAY Calls print on the object
    function disp(obj), obj.display; end
    %DISP Calls print on the object
  end

  methods(Static = true)
  end
end
