# Data Model: Agent-Based RAG Backend Using OpenAI Agents SDK and FastAPI

**Feature**: 2-agent-rag-backend
**Created**: 2025-12-21

## Entity Definitions

### AgentQuery
**Description**: Represents a natural language query submitted to the AI agent for processing

**Attributes**:
- `query_text` (String, required): The original natural language query from the user (1-500 characters)
- `session_id` (String, required): Unique identifier for the conversation session
- `timestamp` (DateTime, required): When the query was submitted (ISO 8601 format)
- `agent_response` (String, optional): The agent's response to the query
- `retrieved_context` (Array<ContextChunk>, optional): Context chunks retrieved for the query
- `query_type` (Enum, optional): Classification of query intent [conceptual, factual, section-based]
- `top_k` (Integer, optional): Number of results requested (default: 5, range: 1-20)
- `min_relevance` (Float, optional): Minimum relevance threshold (default: 0.3, range: 0.0-1.0)

**Validation Rules**:
- `query_text` must be 1-500 characters
- `session_id` must be a valid UUID format
- `timestamp` must be in the past or present
- `top_k` must be between 1 and 20
- `min_relevance` must be between 0.0 and 1.0

### RetrievalTool
**Description**: Interface that allows the agent to access the existing retrieval pipeline

**Attributes**:
- `tool_name` (String, required): Name of the tool ("retrieval_tool")
- `description` (String, required): Description of what the tool does
- `parameters` (Object, required): Schema defining the tool's parameters
- `retrieval_function` (Function, required): The function to call for retrieval

**Validation Rules**:
- `tool_name` must be "retrieval_tool"
- `description` must not be empty
- `parameters` must define a valid JSON schema for the tool

### AgentResponse
**Description**: Output from the AI agent that includes the answer and source attributions

**Attributes**:
- `response_text` (String, required): The agent's generated response (max 2000 characters)
- `source_attributions` (Array<SourceAttribution>, required): Sources used in the response
- `confidence_score` (Float, optional): Agent's confidence in the response (0.0-1.0)
- `processing_time` (Float, required): Time taken to generate the response (seconds)
- `session_id` (String, required): Session identifier for the interaction
- `timestamp` (DateTime, required): When the response was generated (ISO 8601 format)
- `intermediate_steps` (Array<Object>, optional): Steps taken by the agent during processing

**Validation Rules**:
- `response_text` must be 1-2000 characters
- `source_attributions` must contain at least one attribution if content was used
- `confidence_score` must be between 0.0 and 1.0
- `processing_time` must be non-negative
- `session_id` must be a valid UUID format

### SourceAttribution
**Description**: Information about sources used in agent responses

**Attributes**:
- `source_url` (String, required): URL of the source document (valid URL format)
- `section_title` (String, required): Title of the section containing the information
- `chunk_index` (Integer, required): Index of the chunk in the original document (non-negative)
- `relevance_score` (Float, required): Relevance of this source to the query (0.0-1.0)
- `extracted_text` (String, required): The actual text that was used from the source (max 1000 chars)

**Validation Rules**:
- `source_url` must be a valid URL format
- `section_title` must be 1-200 characters
- `chunk_index` must be non-negative
- `relevance_score` must be between 0.0 and 1.0
- `extracted_text` must be 1-1000 characters

### AgentSession
**Description**: Stateful context for a multi-turn conversation with the agent

**Attributes**:
- `session_id` (String, required): Unique identifier for the session (UUID format)
- `created_at` (DateTime, required): When the session was created (ISO 8601 format)
- `last_interaction` (DateTime, required): When the last interaction occurred (ISO 8601 format)
- `conversation_history` (Array<ConversationTurn>, required): History of the conversation
- `active` (Boolean, required): Whether the session is still active (default: true)
- `user_id` (String, optional): Identifier for the user (if authenticated)

**Validation Rules**:
- `session_id` must be a valid UUID format
- `created_at` must be in the past
- `last_interaction` must be after `created_at`
- `conversation_history` items must be valid ConversationTurn objects

### ConversationTurn
**Description**: A single turn in a conversation (query and response pair)

**Attributes**:
- `turn_id` (String, required): Unique identifier for this turn (UUID format)
- `query` (AgentQuery, required): The user's query in this turn
- `response` (AgentResponse, required): The agent's response in this turn
- `timestamp` (DateTime, required): When this turn occurred (ISO 8601 format)

**Validation Rules**:
- `turn_id` must be a valid UUID format
- `timestamp` must be in the past or present
- Both query and response must be valid objects

### ContextChunk
**Description**: Retrieved content chunk with metadata that the agent uses to ground its responses

**Attributes**:
- `chunk_id` (String, required): Unique identifier for the content chunk
- `content` (String, required): The actual text content of the chunk (max 2000 characters)
- `relevance_score` (Float, required): Relevance score of the chunk to the query (0.0-1.0)
- `metadata` (Object, required): Metadata about the chunk including source information
  - `source_url` (String, required): URL of the source document
  - `section_title` (String, required): Title of the section containing the chunk
  - `chunk_index` (Integer, required): Position of chunk in original document
- `rank` (Integer, required): Position in the ranked results list (1-based)

**Validation Rules**:
- `chunk_id` must be unique within the system
- `content` must be 10-2000 characters
- `relevance_score` must be between 0.0 and 1.0
- `rank` must be positive
- All metadata fields must be valid

### ValidationRequest
**Description**: Request to validate that an agent response is properly grounded in retrieved content

**Attributes**:
- `query` (String, required): Original query text
- `response` (String, required): Agent response to validate
- `retrieved_context` (Array<ContextChunk>, required): Context used by the agent
- `expected_sources` (Array<String>, optional): Expected source URLs that should be used

**Validation Rules**:
- `query` must be 1-500 characters
- `response` must be 1-2000 characters
- `retrieved_context` must not be empty

### ValidationResponse
**Description**: Response from the validation service

**Attributes**:
- `is_properly_grounded` (Boolean, required): Whether the response is properly grounded
- `grounding_percentage` (Float, required): Percentage of response grounded in context (0.0-1.0)
- `validation_notes` (String, optional): Additional notes about the validation
- `validation_time` (Float, required): Time taken for validation (seconds)

**Validation Rules**:
- `grounding_percentage` must be between 0.0 and 1.0
- `validation_time` must be non-negative

## Entity Relationships

### AgentQuery → AgentSession
- **Relationship**: Many-to-One
- **Description**: Multiple queries belong to one session
- **Cardinality**: N queries → 1 session

### AgentResponse → AgentSession
- **Relationship**: Many-to-One
- **Description**: Multiple responses belong to one session
- **Cardinality**: N responses → 1 session

### AgentQuery → AgentResponse
- **Relationship**: One-to-One
- **Description**: Each query generates exactly one response
- **Cardinality**: 1 query → 1 response

### AgentSession → ConversationTurn
- **Relationship**: One-to-Many
- **Description**: One session contains multiple conversation turns
- **Cardinality**: 1 session → N turns

### ConversationTurn → AgentQuery
- **Relationship**: One-to-One
- **Description**: Each conversation turn has one query
- **Cardinality**: 1 turn → 1 query

### ConversationTurn → AgentResponse
- **Relationship**: One-to-One
- **Description**: Each conversation turn has one response
- **Cardinality**: 1 turn → 1 response

### AgentResponse → SourceAttribution
- **Relationship**: One-to-Many
- **Description**: One response can reference multiple sources
- **Cardinality**: 1 response → N source attributions

### AgentQuery → ContextChunk
- **Relationship**: One-to-Many
- **Description**: One query retrieves multiple context chunks
- **Cardinality**: 1 query → N context chunks

## State Transitions

### AgentSession States
1. **CREATED**: Session has been initiated but no interactions yet
2. **ACTIVE**: Session is ongoing with recent interactions
3. **INACTIVE**: Session has no recent activity but is not expired
4. **EXPIRED**: Session has exceeded inactivity timeout and is closed
5. **TERMINATED**: Session was explicitly ended by user or system

### Transition Rules
- CREATED → ACTIVE: When first query is received
- ACTIVE → INACTIVE: After inactivity period (e.g., 30 minutes)
- INACTIVE → ACTIVE: When new query is received
- ACTIVE → EXPIRED: After extended inactivity (e.g., 24 hours)
- INACTIVE → EXPIRED: After extended inactivity (e.g., 24 hours)
- Any state → TERMINATED: When explicitly ended

## Data Validation

### Input Validation
- All text inputs are sanitized to prevent injection attacks
- Query lengths are limited to prevent system overload
- Session IDs are validated to be proper UUID format
- URLs are validated to be proper URL format

### Output Validation
- Retrieved results are verified to come from approved content sources
- Source attributions are validated to have complete information
- Agent responses are checked to ensure they reference retrieved content

### Performance Validation
- Response times are monitored and logged
- Quality metrics are calculated for each interaction
- System performance indicators are tracked

### Grounding Validation
- Agent responses are validated against retrieved context
- Hallucination detection algorithms verify content grounding
- Attribution accuracy is measured and logged