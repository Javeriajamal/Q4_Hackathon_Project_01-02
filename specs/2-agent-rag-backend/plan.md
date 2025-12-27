# Implementation Plan: Agent-Based RAG Backend Using OpenAI Agents SDK and FastAPI

**Feature**: 2-agent-rag-backend
**Created**: 2025-12-21
**Status**: Draft
**Input**: User requirements for agent-based RAG system with OpenAI Agents SDK and FastAPI

## Technical Context

### Current State
- RAG ingestion pipeline exists (from spec 1-rag-retrieval)
- Vector search functionality available (Qdrant + Cohere integration)
- Existing retrieval service components in backend/retrieval_service/
- FastAPI framework available for API endpoints
- OpenAI API access available

### Architecture Components
- **Agent Framework**: OpenAI Agents SDK for AI agent creation and management
- **Retrieval Tool**: Wrapper around existing vector search pipeline from spec-2
- **API Layer**: FastAPI endpoints for external agent access
- **Context Injection**: Mechanism to inject retrieved context into agent responses
- **Response Validation**: Grounding verification and fallback handling

### Dependencies
- **OpenAI Python SDK**: For agent creation and management
- **FastAPI**: For API endpoint creation
- **Existing retrieval pipeline**: From spec-2 (Qdrant + Cohere integration)
- **Python Environment**: Python 3.9+

### Technology Stack
- **Backend**: Python 3.9+
- **Agent Framework**: OpenAI Agents SDK
- **API Framework**: FastAPI + Uvicorn
- **Vector DB**: Qdrant (via existing retrieval service)
- **Embeddings**: Cohere (via existing retrieval service)
- **Environment Management**: python-dotenv

### Unknowns
- Specific OpenAI Agents SDK version to use [NEEDS CLARIFICATION]
- Exact agent configuration parameters (model, temperature, etc.) [NEEDS CLARIFICATION]
- Retrieval tool integration pattern with OpenAI Agents [NEEDS CLARIFICATION]

## Constitution Check

### Alignment with Project Principles
- ✅ **Accuracy**: Agent will only respond based on retrieved content (grounded responses)
- ✅ **Clarity**: Responses will include proper source attribution
- ✅ **Consistency**: Uniform response format across all agent interactions
- ✅ **Reproducibility**: Agent responses will be deterministic based on retrieved context
- ✅ **Integrity**: System will avoid hallucinations by grounding responses in source material
- ✅ **Modularity**: Agent functionality will be modular and reusable

### Compliance Verification
- ✅ Agent responses will be limited to textbook content only
- ✅ Proper source attribution will be maintained in all responses
- ✅ Responses will be grounded in retrieved content with validation
- ✅ Error handling will prevent system crashes during service failures

### Potential Violations
- Need to ensure agent doesn't generate responses without sufficient retrieved context
- Must validate that retrieved content is from approved textbook sources only

## Gates

### Gate 1: Technical Feasibility
- **Status**: PASS
- **Verification**: OpenAI Agents SDK is publicly available and documented
- **Dependencies**: API keys available in existing environment setup

### Gate 2: Constitution Compliance
- **Status**: PASS
- **Verification**: Design aligns with project principles of accuracy and integrity
- **Alignment**: Uses existing retrieval pipeline to ensure grounded responses

### Gate 3: Resource Availability
- **Status**: PASS
- **Verification**: All required services have access through existing setup
- **Capacity**: Current infrastructure supports agent-based processing

---

## Phase 0: Outline & Research

### Research Tasks

#### Task 1: OpenAI Agents SDK Integration Pattern
**Objective**: Determine the best approach to integrate OpenAI Agents SDK with existing retrieval pipeline
**Research**: Investigate how to create tools that can access the existing Qdrant/Cohere retrieval functionality

#### Task 2: Agent Configuration Requirements
**Objective**: Identify the optimal agent configuration for RAG applications
**Research**: Best practices for agent parameters, models, and system prompts for RAG use cases

#### Task 3: Context Injection Mechanisms
**Objective**: Understand how to properly inject retrieved context into agent responses
**Research**: Methods for providing retrieved content to agents and ensuring proper grounding

### Research Findings

#### Decision: OpenAI Agents SDK Version
**Rationale**: Using latest stable version of OpenAI Python SDK which includes agent functionality
**Implementation**: Use openai>=1.0.0 which provides agent capabilities through Assistant API

#### Decision: Agent Model Selection
**Rationale**: For RAG applications, GPT-4 or newer models provide best reasoning capabilities
**Options considered**:
- GPT-3.5: Faster but less capable at complex reasoning
- GPT-4: Better at synthesis and reasoning with retrieved content
- GPT-4 Turbo: Good balance of capability and cost
**Chosen**: GPT-4 Turbo (gpt-4-turbo) for optimal performance-cost balance

#### Decision: Retrieval Tool Integration
**Rationale**: Create a custom tool that wraps the existing retrieval service functionality
**Implementation**: Implement a RetrievalTool that calls existing retrieval functions and returns formatted results to the agent

---

## Phase 1: Design & Contracts

### Data Model: data-model.md

#### AgentQuery Entity
- **query_text**: String (required) - The original natural language query from user
- **session_id**: String (required) - Identifier for the conversation session
- **timestamp**: DateTime (required) - When the query was received
- **agent_response**: String (optional) - The agent's response to the query
- **retrieved_context**: List<ContextChunk> (optional) - Context retrieved for the query
- **query_type**: Enum (conceptual, factual, section-based) - Classification of query intent

#### RetrievalTool Entity
- **tool_name**: String (required) - Name of the tool ("retrieval_tool")
- **description**: String (required) - Description of what the tool does
- **parameters**: Object (required) - Schema defining the tool's parameters
- **retrieval_function**: Function (required) - The function to call for retrieval

#### AgentResponse Entity
- **response_text**: String (required) - The agent's generated response
- **source_attributions**: List<SourceAttribution> (required) - Sources used in the response
- **confidence_score**: Float (optional) - Agent's confidence in the response
- **processing_time**: Float (required) - Time taken to generate the response
- **session_id**: String (required) - Session identifier for the interaction
- **timestamp**: DateTime (required) - When the response was generated

#### SourceAttribution Entity
- **source_url**: String (required) - URL of the source document
- **section_title**: String (required) - Title of the section containing the information
- **chunk_index**: Integer (required) - Index of the chunk in the original document
- **relevance_score**: Float (required) - Relevance of this source to the query (0.0-1.0)
- **extracted_text**: String (required) - The actual text that was used from the source

#### AgentSession Entity
- **session_id**: String (required) - Unique identifier for the session
- **created_at**: DateTime (required) - When the session was created
- **last_interaction**: DateTime (required) - When the last interaction occurred
- **conversation_history**: List<ConversationTurn> (required) - History of the conversation
- **active**: Boolean (required) - Whether the session is still active

#### ConversationTurn Entity
- **turn_id**: String (required) - Unique identifier for this turn
- **query**: AgentQuery (required) - The user's query
- **response**: AgentResponse (required) - The agent's response
- **timestamp**: DateTime (required) - When this turn occurred

### API Contracts: contracts/agent-api.yaml

#### Endpoint: POST /api/agent/query
**Purpose**: Submit a query to the AI agent for processing with retrieval-augmented generation

**Request Body**:
```json
{
  "query": "Natural language query text",
  "session_id": "unique-session-identifier",
  "top_k": 5,
  "min_relevance": 0.3
}
```

**Response**:
```json
{
  "response": "Generated response based on retrieved context",
  "source_attributions": [
    {
      "source_url": "https://physicalairobotics.netlify.app/docs/module-2-digital-twin-simulation/chapter-1-introduction",
      "section_title": "Introduction to Digital Twin Simulation",
      "chunk_index": 2,
      "relevance_score": 0.85,
      "extracted_text": "Digital twin simulation is a virtual representation of a physical system..."
    }
  ],
  "confidence_score": 0.92,
  "processing_time": 2.34,
  "session_id": "unique-session-identifier",
  "timestamp": "2025-12-21T01:00:00Z"
}
```

**Error Responses**:
- 400: Invalid query parameters
- 429: Rate limit exceeded (from OpenAI or Cohere)
- 500: Internal server error (agent or retrieval failure)

#### Endpoint: POST /api/agent/validate-response
**Purpose**: Validate that an agent response is properly grounded in retrieved content

**Request Body**:
```json
{
  "query": "Original query text",
  "response": "Agent response to validate",
  "retrieved_context": [
    {
      "content": "Retrieved content chunk",
      "source_url": "https://example.com",
      "relevance_score": 0.8
    }
  ]
}
```

**Response**:
```json
{
  "is_properly_grounded": true,
  "grounding_percentage": 0.85,
  "validation_notes": "Response appropriately uses retrieved content",
  "validation_time": 0.12
}
```

### Quickstart Guide: quickstart.md

#### Setting up Agent-Based RAG Service

1. **Prerequisites**
   - Python 3.9+
   - OpenAI API key
   - Cohere API key (from existing setup)
   - Qdrant Cloud instance access (from existing setup)
   - Environment variables configured

2. **Environment Setup**
   ```bash
   # Set environment variables
   export OPENAI_API_KEY="your_openai_api_key"
   export COHERE_API_KEY="from_existing_setup"
   export QDRANT_URL="from_existing_setup"
   export QDRANT_API_KEY="from_existing_setup"
   ```

3. **Install Dependencies**
   ```bash
   pip install openai fastapi uvicorn python-dotenv
   ```

4. **Run Agent Service**
   ```bash
   cd backend
   uvicorn agent_service.main:app --host 0.0.0.0 --port 8001
   ```

5. **Test Agent Query**
   ```bash
   curl -X POST http://localhost:8001/api/agent/query \
     -H "Content-Type: application/json" \
     -d '{
       "query": "Explain digital twin simulation concepts",
       "top_k": 5
     }'
   ```

## Phase 2: Implementation Plan

### Implementation Components

#### 1. Agent Initialization Module
- **Purpose**: Initialize OpenAI agent with proper configuration
- **Location**: `agent_service/agent.py`
- **Dependencies**: OpenAI SDK
- **Functions**:
  - `initialize_agent(model_name: str, system_prompt: str) -> Assistant`
  - `create_agent_session(initial_context: str) -> str`

#### 2. Retrieval Tool Module
- **Purpose**: Wrap existing retrieval functionality as an agent tool
- **Location**: `agent_service/retrieval_tool.py`
- **Dependencies**: Existing retrieval service
- **Functions**:
  - `create_retrieval_tool() -> Tool`
  - `retrieve_context(query: str, top_k: int, min_relevance: float) -> List[Dict]`

#### 3. Context Injection Module
- **Purpose**: Inject retrieved context into agent responses
- **Location**: `agent_service/context_injector.py`
- **Functions**:
  - `inject_context_into_agent(agent: Assistant, context: List[Dict]) -> Assistant`
  - `format_context_for_agent(context_chunks: List[RetrievalResult]) -> str`

#### 4. Response Validation Module
- **Purpose**: Validate that responses are properly grounded
- **Location**: `agent_service/validation.py`
- **Functions**:
  - `validate_response_grounding(response: str, context: List[Dict]) -> bool`
  - `calculate_grounding_score(response: str, context: List[Dict]) -> float`

#### 5. API Service Module
- **Purpose**: Expose agent functionality via FastAPI
- **Location**: `agent_service/api.py`
- **Dependencies**: FastAPI
- **Functions**:
  - `agent_query_endpoint(query_request: AgentQueryRequest) -> AgentQueryResponse`

### Development Tasks

1. **Setup Phase**:
   - Create agent_service directory structure
   - Set up environment configuration for OpenAI
   - Install required dependencies (openai SDK)

2. **Foundational Phase**:
   - Create data models for agent interactions
   - Implement agent initialization with OpenAI SDK
   - Create retrieval tool wrapper for existing pipeline

3. **Core Functionality Phase**:
   - Implement context injection mechanisms
   - Build API endpoints for agent queries
   - Add response validation and grounding checks

4. **Validation Phase**:
   - Create validation endpoints
   - Add comprehensive error handling
   - Implement logging and monitoring

5. **Polish Phase**:
   - Add performance optimizations
   - Create comprehensive test suite
   - Update documentation

## Constitution Re-Check (Post-Design)

### Updated Compliance Verification
- ✅ Agent responses limited to textbook content via retrieval tool
- ✅ Proper source attribution maintained in all responses
- ✅ Responses grounded in retrieved content with validation
- ✅ System prevents hallucinations through grounding validation
- ✅ Deterministic behavior based on retrieved context

### Architecture Alignment
- ✅ Uses existing retrieval pipeline to ensure content accuracy
- ✅ Modular design supports future enhancements
- ✅ Performance requirements can be met with agent framework
- ✅ Logging and monitoring support quality assurance requirements

## Next Steps

### Immediate Actions
1. Implement agent initialization module with OpenAI SDK
2. Create retrieval tool wrapper for existing pipeline
3. Build context injection mechanisms
4. Develop API endpoints for agent queries
5. Implement response validation and grounding checks

### Success Criteria Validation
- [ ] Agent can process natural language queries with retrieved context
- [ ] Responses include proper source attribution (URL, section, chunk index)
- [ ] Agent avoids hallucinations when no relevant context is found
- [ ] Response time stays under 10 seconds for 95% of requests
- [ ] System handles 50 concurrent queries without degradation