%class DopplerFactorWithClockV, see Doxygen page for details
%at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
%
%-------Constructors-------
%DopplerFactorWithClockV(size_t keyV, size_t keyD, Vector losvec, Vector prr, Vector orgv, Base model)
%
classdef DopplerFactorWithClockV < gtsam.NoiseModelFactor
  properties
    ptr_gtsam_gnssDopplerFactorWithClockV = 0
  end
  methods
    function obj = DopplerFactorWithClockV(varargin)
      if (nargin == 2 || (nargin == 3 && strcmp(varargin{3}, 'void'))) && isa(varargin{1}, 'uint64') && varargin{1} == uint64(5139824614673773682)
        if nargin == 2
          my_ptr = varargin{2};
        else
          my_ptr = gtsam_gnss_wrapper(45, varargin{2});
        end
        base_ptr = gtsam_gnss_wrapper(44, my_ptr);
      elseif nargin == 6 && isa(varargin{1},'numeric') && isa(varargin{2},'numeric') && isa(varargin{3},'double') && isa(varargin{4},'double') && isa(varargin{5},'double') && isa(varargin{6},'gtsam.noiseModel.Base')
        [ my_ptr, base_ptr ] = gtsam_gnss_wrapper(46, varargin{1}, varargin{2}, varargin{3}, varargin{4}, varargin{5}, varargin{6});
      else
        error('Arguments do not match any overload of gtsam_gnss.DopplerFactorWithClockV constructor');
      end
      obj = obj@gtsam.NoiseModelFactor(uint64(5139824614673773682), base_ptr);
      obj.ptr_gtsam_gnssDopplerFactorWithClockV = my_ptr;
    end

    function delete(obj)
      gtsam_gnss_wrapper(47, obj.ptr_gtsam_gnssDopplerFactorWithClockV);
    end

    function display(obj), obj.print(''); end
    %DISPLAY Calls print on the object
    function disp(obj), obj.display; end
    %DISP Calls print on the object
  end

  methods(Static = true)
  end
end
