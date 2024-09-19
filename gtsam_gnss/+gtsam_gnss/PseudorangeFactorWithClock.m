%class PseudorangeFactorWithClock, see Doxygen page for details
%at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html
%
%-------Constructors-------
%PseudorangeFactorWithClock(size_t keyX, size_t keyC, Vector losvec, Vector pr, int sigtype, Vector orgx, Base model)
%
classdef PseudorangeFactorWithClock < gtsam.NoiseModelFactor
  properties
    ptr_gtsam_gnssPseudorangeFactorWithClock = 0
  end
  methods
    function obj = PseudorangeFactorWithClock(varargin)
      if (nargin == 2 || (nargin == 3 && strcmp(varargin{3}, 'void'))) && isa(varargin{1}, 'uint64') && varargin{1} == uint64(5139824614673773682)
        if nargin == 2
          my_ptr = varargin{2};
        else
          my_ptr = gtsam_gnss_wrapper(29, varargin{2});
        end
        base_ptr = gtsam_gnss_wrapper(28, my_ptr);
      elseif nargin == 7 && isa(varargin{1},'numeric') && isa(varargin{2},'numeric') && isa(varargin{3},'double') && isa(varargin{4},'double') && isa(varargin{5},'numeric') && isa(varargin{6},'double') && isa(varargin{7},'gtsam.noiseModel.Base')
        [ my_ptr, base_ptr ] = gtsam_gnss_wrapper(30, varargin{1}, varargin{2}, varargin{3}, varargin{4}, varargin{5}, varargin{6}, varargin{7});
      else
        error('Arguments do not match any overload of gtsam_gnss.PseudorangeFactorWithClock constructor');
      end
      obj = obj@gtsam.NoiseModelFactor(uint64(5139824614673773682), base_ptr);
      obj.ptr_gtsam_gnssPseudorangeFactorWithClock = my_ptr;
    end

    function delete(obj)
      gtsam_gnss_wrapper(31, obj.ptr_gtsam_gnssPseudorangeFactorWithClock);
    end

    function display(obj), obj.print(''); end
    %DISPLAY Calls print on the object
    function disp(obj), obj.display; end
    %DISP Calls print on the object
  end

  methods(Static = true)
  end
end
