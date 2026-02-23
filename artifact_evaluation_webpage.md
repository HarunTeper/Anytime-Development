Artifact Evaluation

Artifact Evaluation (AE) for RTAS is an optional evaluation process for research works that have been accepted for publication at RTAS. Due to the systems-oriented scope of RTAS, authors are strongly encouraged to submit artifacts to increase the value and the visibility of their research.

The AE process seeks to further the goal of reproducible science. It offers authors the opportunity to highlight the reproducibility of their results, and to obtain a validation given by the community for the experiments and data reported in their paper. In the AE process, peer practitioners from the community will follow the instructions included in the artifacts and give feedback to the authors, while keeping papers and artifacts confidential and under the control of the authors.

The AE process is non-competitive and success-oriented. The acceptance of the papers has already been decided before the AE process starts and the hope is that all submitted artifacts will pass the evaluation criteria. Authors of artifacts are expected to provide detailed and reasonable procedures for the evaluators to interact with the evaluators to fix any possible technical issues that may emerge during the review process and to improve the portability of the artifact.

Authors of papers corresponding to artifacts that pass the evaluation will be entitled to include, in the camera-ready version of their paper, an RTAS AE seal that indicates that the artifact has passed the repeatability test. Authors are also entitled (and indeed encouraged) to also use this RTAS AE seal on the title slide of the corresponding presentation at RTAS’25.

All (conditionally) accepted RTAS papers are encouraged to participate in AE. The evaluators may give early feedback if there are any issues with the artifacts that prevent them from being run correctly. See below for details on the (expected) submission timeline and process, and evaluation criteria.
Important Dates

All times are UTC-12, or “anywhere on earth,” unless otherwise stated.

    RTAS acceptance notification: January 29, 2026
    AE submission deadline: February 20 February 26, 2026
    AE notification: March 20, 2026
    RTAS Paper Camera-ready deadline: March 31, 2026

Organizers
Chair:

    Filip Marković, University of Southampton, UK

Program Committee:

    Anam Farukh, Boston University, USA
    Binqi Sun, TUM, Germany
    Harun Teper, TU Dortmund, Germany
    Joshua Bakita, Mohamed bin Zayed University of Artifical Intelligence, UAE
    Lara Orlandić, EPFL, Switzerland
    Marion Sudvarg, Washington University in St. Louis, USA
    Sergi Vilardell, Barcelona Supercomputing Center, Spain
    Shareef Ahmed, University of South Florida, USA
    Vijay Banarjee, Washington State University, USA
    Zenepe Satka, Mälardalen University, Sweden

Artifact Submission

Submit to Artifact Evaluation

Simply register at the above link and then click "New submission."

When submitting an artifact for evaluation, please provide a document (e.g., a PDF, HTML, or text file) with instructions on:

    the system requirements;
    how to use the packaged artifact (please reference specific figures and tables in the paper that will be reproduced); and
    how to set up the artifact on a machine different from the provided packaged artifact (e.g., specific versions of software to install on a clean machine);

to reproduce the results in the paper. The document should include a link to the packaged artifact (e.g., a virtual machine image) and a description of key configuration parameters for the packaged artifact (including hardware characteristics needed such as type of architecture, RAM, number of cores, etc.). To protect evaluator anonymity, please make sure that the website/server for downloading the packaged artifact does not include visitor tracking or analytics code during the artifact evaluation period.

Please provide precise instructions on how to proceed after booting the image, including the instructions for running the artifact. Authors are strongly encouraged to prepare readable scripts to launch the experiments automatically. In the case the experimentations require a long time to complete, the authors may prepare simplified experiments (e.g., by reducing the number of samples over which the results are averaged) with shorter running times that demonstrate the same trends as observed in the complete experiments (and as reported in the paper).

Finally, be sure to include a version of the accepted paper related to the artifact that is as close as possible to the final camera-ready version.

A good “how-to” guide for preparing an effective artifact evaluation package is available online at http://bit.ly/HOWTO-AEC.
Artifacts with Special Handling Requirements

If your artifact requires:

    special hardware (e.g., embedded boards, FPGA, or cloud infrastructure),
    commercial software (e.g., proprietary RTOS, middleware, or tools like MATLAB), or
    proprietary data that cannot be distributed to the AE committee,

please consider how you can make it available to evaluators online. For instance, providing evaluators with remote access to your hardware setup could be an option. If you are unsure, contact the AE chair as soon as possible.

To maintain the anonymity of artifact evaluators, authors should not embed any visitor tracking or analytics code in the website/server for their artifacts.
Artifact Packaging Recommendations

Based on previous experience, the biggest hurdle to successful reproducibility is the setup and installation of the necessary libraries and dependencies. Authors are therefore encouraged to prepare a virtual machine (VM) image including their artifact (if possible) and to make it available via HTTP throughout the evaluation process (and, ideally, afterward). As the basis of the VM image, please choose commonly-used OS versions that have been tested with the virtual machine software and that evaluators are likely to be accustomed to. We encourage authors to use VirtualBox (https://www.virtualbox.org) and save the VM image as an Open Virtual Appliance (OVA) file. To facilitate the preparation of the VM, we suggest using the VM images available at https://www.osboxes.org/.
Accepted Artifact Examples

    Demystifiying NVIDIA GPU Internals to Enable Reliable GPU Management, RTAS 2024 (website)
    Probabilistic Response-Time-Aware Search for Transient Astrophysical Phenomena, RTSS 2025 (Github)
    Timing Analysis of Asynchronized Distributed Cause-Effect Chains, RTAS 2021 (Github)

Evaluation Criteria

Submitted artifacts will be judged based on three criteria, namely coverage for repeatability, instructions, and quality, (see more details below), where each criterion is assessed on the following five-point scale:

    significantly exceeds expectations (5),
    exceeds expectations (4),
    meets expectations (3),
    falls below expectations (2),
    missing or significantly falls below expectations (1).

To pass the AE process, an artifact must generally “meet expectations” (average score of 3 or more), and must not have any missing elements (no scores of 1). Each artifact is evaluated independently according to the listed objective criteria. The higher scores (“exceeds” or “significantly exceeds expectations”) in the criteria will be considered aspirational goals, not requirements for acceptance.

Artifact evaluation is single-blind. Artifacts will be held in confidence by the evaluation committee. While the committee strongly encourages the authors of RTAS papers to make their artifacts publicly available, the artifact evaluation process is open to artifacts that are not.
Criterion 1: Coverage for Repeatability

The focus is on figures or tables in the paper containing computationally generated or processed experimental evidence used to support the claims of the paper. Other figures and tables, such as illustrations or tables listing only parameter values, are not considered in this calculation.

Note that satisfying this criterion does not require that the corresponding figures or tables be recreated in exactly the same format as appears in the paper, merely that the data underlying those figures or tables be generated faithfully in a recognizable format.

A repeatable element is one for which the computation can be rerun by following the instructions provided with the artifact in a suitably equipped environment.

The categories for this criterion are:

    None (missing / 1): There are no repeatable elements.
    Some (falls below expectations / 2): There is at least one repeatable element.
    Most (meets expectations / 3): The majority (at least half) of the elements are repeatable.
    All repeatable or most extensible (exceeds expectations / 4): All elements are repeatable or most are repeatable and easily modified. Note that if there is only one computational element and it is repeatable, then this score should be awarded.
    All extensible (significantly exceeds expectations / 5): All elements are repeatable and easily modified.

Criterion 2: Instruction

Instructions are intended for other practitioners that be able to recreate the paper’s computationally generated results. The categories for this criterion are:

    None (missing / 1): No instructions were included in the artifact.
    Rudimentary (falls below expectations / 2): The instructions specify a script or command to run, but little else.
    Complete (meets expectations / 3): For every computational element that is repeatable, there is a specific instruction that explains how to repeat it. The environment under which the software was originally run is described.
    Comprehensive (exceeds expectations / 4): For every computational element that is repeatable there is a single command or clearly defined short series of steps that recreates that element almost exactly as it appears in the published paper (e.g., file format, fonts, line styles, etc. might not be the same, but the content of the element is the same). In addition to identifying the specific environment under which the software was originally run, a broader class of environments is identified under which it could run.
    Outstanding (significantly exceeds expectations / 5): In addition to the criteria for a comprehensive set of instructions, explanations are provided of:
        all the major components/modules in the software,
        important design decisions made during implementation,
        how to modify/extend the software, and/or
        what environments/modifications would break the software.

Criterion 3: Quality

This criterion explores the means provided to infer, show, or prove the trustworthiness of the software and its results. While a set of scripts that exactly recreate, for example, the figures from the paper certainly aid in repeatability, without well-documented code it is hard to understand how the data in that figure was processed, without well-documented data it is hard to determine whether the input is correct, and without testing it is hard to determine whether the results can be trusted.

If there are tests in the artifact which are not included in the paper, they should at least be mentioned in the instructions document. Documentation of test details can be put into the instructions document or into a separate document in the artifact.

The categories for this criterion are:

    None (missing / 1): There is no evidence of software documentation or testing.
    Rudimentary documentation (falls below expectations / 2): The purpose of almost all files is documented (preferably within the file, but otherwise in the instructions or a separate README file).
    Comprehensive documentation (meets expectations / 3): The purpose of almost all files is documented. Within source code files, almost all classes, methods, attributes, and variables are given lengthy clear names and/or documentation of their purpose. Within data files, the format and structure of the data is documented; for example, in comma-separated value (CSV) files there is a header row and/or comments explaining the contents of each column.
    Comprehensive documentation and rudimentary testing (exceeds expectations / 4): In addition to the criteria for comprehensive documentation, there are identified test cases with known solutions that can be run to validate at least some components of the code.
    Comprehensive documentation and testing (significantly exceeds expectations / 5): In addition to the criteria for comprehensive documentation, there are clearly identified unit tests (preferably run within a unit test framework) which exercise a significant fraction of the smaller components of the code (individual functions and classes) and system-level tests which exercise a significant fraction of the full package. Unit tests are typically self-documenting, but the system-level tests will require documentation of at least the source of the known solution.
